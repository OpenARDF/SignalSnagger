#!/usr/bin/env python3
"""Independently validate a SignalSnagger single-target release package."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import zipfile
from pathlib import Path


FLASH_BYTES = 131072


def fail(message: str) -> None:
    raise RuntimeError(message)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def one_match(directory: Path, pattern: str, description: str) -> Path:
    matches = [path for path in directory.iterdir() if re.fullmatch(pattern, path.name)]
    if len(matches) != 1:
        fail(f"expected exactly one {description}; found {len(matches)}")
    return matches[0]


def parse_checksums(path: Path) -> dict[str, str]:
    checksums: dict[str, str] = {}
    for line_number, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        match = re.fullmatch(r"([0-9a-f]{64})  ([^/]+)", line)
        if not match or match.group(2) in checksums:
            fail(f"{path.name}:{line_number}: invalid checksum entry")
        checksums[match.group(2)] = match.group(1)
    return checksums


def parse_hex(path: Path) -> dict[int, int]:
    image: dict[int, int] = {}
    base = 0
    saw_eof = False
    for line_number, raw_line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw_line.strip()
        if saw_eof:
            fail(f"{path.name}:{line_number}: record follows Intel HEX EOF")
        if not line.startswith(":"):
            fail(f"{path.name}:{line_number}: invalid Intel HEX record")
        try:
            record = bytes.fromhex(line[1:])
        except ValueError as error:
            fail(f"{path.name}:{line_number}: non-hexadecimal record")
        if len(record) < 5 or record[0] + 5 != len(record) or sum(record) & 0xFF:
            fail(f"{path.name}:{line_number}: invalid Intel HEX record")
        count = record[0]
        offset = (record[1] << 8) | record[2]
        record_type = record[3]
        data = record[4 : 4 + count]
        if record_type == 0x00:
            for index, value in enumerate(data):
                address = base + offset + index
                if address in image and image[address] != value:
                    fail(f"{path.name}: conflicting data at 0x{address:X}")
                image[address] = value
        elif record_type == 0x01:
            if count != 0 or offset != 0:
                fail(f"{path.name}:{line_number}: invalid Intel HEX EOF record")
            saw_eof = True
        elif record_type == 0x02 and count == 2:
            base = int.from_bytes(data, "big") << 4
        elif record_type == 0x04 and count == 2:
            base = int.from_bytes(data, "big") << 16
        elif record_type not in (0x03, 0x05):
            fail(f"{path.name}:{line_number}: unsupported Intel HEX record")
    if not saw_eof or not image:
        fail(f"{path.name}: incomplete or empty Intel HEX image")
    return image


def require_string(data: dict[str, object], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value:
        fail(f"release info field {key} must be a non-empty string")
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-dir", required=True, type=Path)
    parser.add_argument(
        "--require-clean",
        action="store_true",
        help="Reject a package whose recorded source tree was dirty",
    )
    options = parser.parse_args()
    package = options.package_dir.resolve()
    if not package.is_dir():
        fail(f"package directory does not exist: {package}")

    info_path = one_match(
        package,
        r"SignalSnagger-Release-Info-v\d+\.\d+\.\d+\.json",
        "release-info file",
    )
    release_info = json.loads(info_path.read_text(encoding="utf-8"))
    if release_info.get("format") != "signalsnagger-release-info-v1":
        fail("unsupported release-info format")
    if release_info.get("product") != "SignalSnagger":
        fail("unexpected product")
    version = require_string(release_info, "version")
    if not re.fullmatch(r"\d+\.\d+\.\d+", version):
        fail(f"invalid release version: {version}")
    if release_info.get("hardwareVariant") != "AVR128DA48":
        fail("unexpected hardware variant")
    if release_info.get("mcu") != "avr128da48":
        fail("unexpected MCU")
    if not re.fullmatch(r"[0-9a-f]{40}", require_string(release_info, "gitCommit")):
        fail("gitCommit is not a full lowercase Git commit ID")
    if not re.fullmatch(
        r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z",
        require_string(release_info, "generatedUtc"),
    ):
        fail("generatedUtc is not a UTC timestamp")
    if not isinstance(release_info.get("sourceTreeDirty"), bool):
        fail("sourceTreeDirty must be Boolean")
    if options.require_clean and release_info["sourceTreeDirty"]:
        fail("release package was built from a dirty source tree")
    build_profile = release_info.get("buildProfile")
    if not isinstance(build_profile, dict):
        fail("release info is missing build-profile metadata")
    if build_profile.get("status") != "reference-version-match":
        fail("release package was not built with the reference profile")

    names = {
        "firmware": f"SignalSnagger-v{version}.hex",
        "release_info": f"SignalSnagger-Release-Info-v{version}.json",
        "checksums": f"SignalSnagger-Checksums-v{version}.txt",
        "readme": f"README-SignalSnagger-v{version}.txt",
        "zip": f"SignalSnagger-v{version}-Release-Files.zip",
    }
    actual_names = {path.name for path in package.iterdir() if path.is_file()}
    if actual_names != set(names.values()):
        fail(f"package files do not match the contract: {sorted(actual_names)}")

    checksums = parse_checksums(package / names["checksums"])
    checksum_names = {names["firmware"], names["release_info"], names["readme"]}
    if set(checksums) != checksum_names:
        fail("checksum file does not list the exact package payload")
    for name, expected in checksums.items():
        if sha256(package / name) != expected:
            fail(f"checksum mismatch for {name}")

    files = release_info.get("files")
    if not isinstance(files, list) or len(files) != 1 or not isinstance(files[0], dict):
        fail("release info must contain exactly one firmware manifest entry")
    entry = files[0]
    firmware_path = package / names["firmware"]
    if entry.get("fileName") != names["firmware"] or entry.get("kind") != "firmware":
        fail("firmware manifest entry has unexpected identity")
    if entry.get("sizeBytes") != firmware_path.stat().st_size:
        fail("firmware manifest size mismatch")
    if entry.get("sha256") != sha256(firmware_path):
        fail("firmware manifest hash mismatch")

    image = parse_hex(firmware_path)
    if min(image) != 0 or max(image) >= FLASH_BYTES:
        fail("firmware image is outside AVR128DA48 flash")
    padded = bytearray([0xFF]) * FLASH_BYTES
    for address, value in image.items():
        padded[address] = value
    firmware = release_info.get("firmware")
    if not isinstance(firmware, dict):
        fail("release info is missing firmware metadata")
    expected_firmware = {
        "fileName": names["firmware"],
        "startAddress": f"0x{min(image):X}",
        "lastAddress": f"0x{max(image):X}",
        "bytesInImage": len(image),
        "sha256": sha256(firmware_path),
        "fullFlashSha256": sha256_bytes(padded),
    }
    if firmware != expected_firmware:
        fail("firmware metadata does not match the Intel HEX image")
    if version.encode("ascii") + b"\x00" not in padded:
        fail("firmware image does not contain the exact null-terminated release version")
    if re.search(re.escape(version.encode("ascii")) + rb"[a-z]", padded):
        fail("firmware image still contains a suffixed test version")

    archive_names = {
        names["firmware"],
        names["release_info"],
        names["checksums"],
        names["readme"],
    }
    with zipfile.ZipFile(package / names["zip"]) as archive:
        members = archive.namelist()
        if len(members) != len(set(members)) or set(members) != archive_names:
            fail("release ZIP contents do not exactly match the package contract")
        for name in members:
            if Path(name).name != name or archive.read(name) != (package / name).read_bytes():
                fail(f"release ZIP member mismatch: {name}")

    print(f"PASS SignalSnagger {version} release package: {package}")
    print(f"{sha256(firmware_path)}  {names['firmware']}")
    print(f"{sha256(package / names['zip'])}  {names['zip']}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError, zipfile.BadZipFile) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2) from error
