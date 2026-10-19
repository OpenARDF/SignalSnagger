#!/usr/bin/env python3
"""Build a deterministic single-target SignalSnagger release package."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path


FLASH_BYTES = 131072
MCU = "avr128da48"
HARDWARE_VARIANT = "AVR128DA48"


def fail(message: str) -> None:
    raise RuntimeError(message)


def run(command: list[str], root: Path, *, capture: bool = False) -> str:
    result = subprocess.run(
        command,
        cwd=root,
        check=True,
        text=True,
        stdout=subprocess.PIPE if capture else None,
    )
    return result.stdout.strip() if capture else ""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


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
            fail(f"{path.name}:{line_number}: non-hexadecimal Intel HEX record")
        if len(record) < 5 or record[0] + 5 != len(record):
            fail(f"{path.name}:{line_number}: invalid Intel HEX length")
        if sum(record) & 0xFF:
            fail(f"{path.name}:{line_number}: invalid Intel HEX checksum")
        count = record[0]
        offset = (record[1] << 8) | record[2]
        record_type = record[3]
        data = record[4 : 4 + count]
        if record_type == 0x00:
            for index, value in enumerate(data):
                address = base + offset + index
                previous = image.get(address)
                if previous is not None and previous != value:
                    fail(f"{path.name}: conflicting data at address 0x{address:X}")
                image[address] = value
        elif record_type == 0x01:
            if count != 0 or offset != 0:
                fail(f"{path.name}:{line_number}: invalid Intel HEX EOF record")
            saw_eof = True
        elif record_type == 0x02:
            if count != 2:
                fail(f"{path.name}:{line_number}: invalid segment-address record")
            base = int.from_bytes(data, "big") << 4
        elif record_type == 0x04:
            if count != 2:
                fail(f"{path.name}:{line_number}: invalid linear-address record")
            base = int.from_bytes(data, "big") << 16
        elif record_type not in (0x03, 0x05):
            fail(f"{path.name}:{line_number}: unsupported Intel HEX record type")
    if not saw_eof or not image:
        fail(f"{path.name}: incomplete or empty Intel HEX image")
    return image


def full_flash_sha256(image: dict[int, int]) -> str:
    padded = bytearray([0xFF]) * FLASH_BYTES
    for address, value in image.items():
        if address >= FLASH_BYTES:
            fail(f"flash data exceeds AVR128DA48 memory at 0x{address:X}")
        padded[address] = value
    return hashlib.sha256(padded).hexdigest()


def package_entry(path: Path, kind: str, purpose: str) -> dict[str, object]:
    return {
        "fileName": path.name,
        "kind": kind,
        "purpose": purpose,
        "sizeBytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def zip_info(name: str, timestamp: datetime) -> zipfile.ZipInfo:
    # ZIP timestamps have two-second resolution; fixed metadata makes repeats identical.
    info = zipfile.ZipInfo(name, timestamp.timetuple()[:6])
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o100644 << 16
    info.create_system = 3
    return info


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-root",
        type=Path,
        help="Parent directory for the versioned package directory",
    )
    options = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    firmware_root = root / "Software" / "AVR128DA48" / "SignalSnagger"
    defs = (firmware_root / "defs.h").read_text(encoding="utf-8")
    match = re.search(r'^#define SW_REVISION "([^"]+)"$', defs, re.MULTILINE)
    if not match:
        fail("defs.h does not define SW_REVISION")
    version = match.group(1)
    if not re.fullmatch(r"\d+\.\d+\.\d+", version):
        fail(f"release packaging requires an unsuffixed x.y.z version; found {version}")

    toolchain_root = os.environ.get("AVR_TOOLCHAIN_ROOT")
    dfp_root = os.environ.get("AVR_DFP_ROOT")
    if not toolchain_root or not dfp_root:
        fail("AVR_TOOLCHAIN_ROOT and AVR_DFP_ROOT must be set")

    run([str(root / "scripts" / "avr-build.sh")], root)
    source_hex = root / "build" / "avr" / "SignalSnagger.hex"
    image = parse_hex(source_hex)
    first_address = min(image)
    last_address = max(image)
    if first_address != 0 or last_address >= FLASH_BYTES:
        fail("firmware image is outside AVR128DA48 flash")

    output_parent = (options.output_root or root / "build" / "release-packages").resolve()
    output = output_parent / f"SignalSnagger-{version}"
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)

    friendly_version = f"v{version}"
    names = {
        "firmware": f"SignalSnagger-{friendly_version}.hex",
        "release_info": f"SignalSnagger-Release-Info-{friendly_version}.json",
        "checksums": f"SignalSnagger-Checksums-{friendly_version}.txt",
        "readme": f"README-SignalSnagger-{friendly_version}.txt",
        "zip": f"SignalSnagger-{friendly_version}-Release-Files.zip",
    }
    paths = {key: output / name for key, name in names.items()}
    shutil.copyfile(source_hex, paths["firmware"])

    readme = f"""SignalSnagger {friendly_version}

Hardware variant: {HARDWARE_VARIANT}

Firmware:
- {names['firmware']}

Metadata and integrity:
- {names['release_info']}
- {names['checksums']}

SignalSnagger has no resident bootloader or software updater. Program the
firmware with a compatible UPDI programmer using an isolated flash erase so
EEPROM and configuration memories remain intact, then verify by readback.

Upload {names['firmware']} by itself for the firmware image.
Upload {names['zip']} for the complete one-variant release bundle.
"""
    paths["readme"].write_text(readme, encoding="utf-8")

    source_commit = run(["git", "rev-parse", "HEAD"], root, capture=True)
    commit_time = run(["git", "show", "-s", "--format=%cI", "HEAD"], root, capture=True)
    timestamp = datetime.fromisoformat(commit_time).astimezone(timezone.utc)
    generated = timestamp.isoformat().replace("+00:00", "Z")
    source_dirty = bool(run(["git", "status", "--porcelain"], root, capture=True))
    compiler = run(
        [str(Path(toolchain_root) / "bin" / "avr-g++"), "--version"],
        root,
        capture=True,
    ).splitlines()[0]
    pdsc = (Path(dfp_root) / "Atmel.AVR-Dx_DFP.pdsc").read_text(encoding="utf-8")
    dfp_match = re.search(r'<release version="([^"]+)"', pdsc)
    if not dfp_match:
        fail("AVR-Dx DFP manifest does not contain a release version")

    firmware_entry = package_entry(
        paths["firmware"],
        "firmware",
        "Complete AVR128DA48 receiver firmware for programmer installation.",
    )
    release_info = {
        "format": "signalsnagger-release-info-v1",
        "product": "SignalSnagger",
        "version": version,
        "hardwareVariant": HARDWARE_VARIANT,
        "mcu": MCU,
        "generatedUtc": generated,
        "gitCommit": source_commit,
        "sourceTreeDirty": source_dirty,
        "buildProfile": {
            "status": "reference-version-match",
            "compilerVersion": compiler,
            "dfpVersion": dfp_match.group(1),
            "warningCount": 0,
            "maximumFlashBytes": 120000,
            "maximumStaticRamBytes": 12288,
        },
        "firmware": {
            "fileName": names["firmware"],
            "startAddress": f"0x{first_address:X}",
            "lastAddress": f"0x{last_address:X}",
            "bytesInImage": len(image),
            "sha256": sha256(paths["firmware"]),
            "fullFlashSha256": full_flash_sha256(image),
        },
        "files": [firmware_entry],
    }
    paths["release_info"].write_text(
        f"{json.dumps(release_info, indent=2)}\n",
        encoding="utf-8",
    )

    checksum_paths = [paths["firmware"], paths["release_info"], paths["readme"]]
    paths["checksums"].write_text(
        "".join(f"{sha256(path)}  {path.name}\n" for path in checksum_paths),
        encoding="ascii",
    )

    archive_paths = [
        paths["firmware"],
        paths["release_info"],
        paths["checksums"],
        paths["readme"],
    ]
    with zipfile.ZipFile(paths["zip"], "w") as archive:
        for path in archive_paths:
            archive.writestr(
                zip_info(path.name, timestamp),
                path.read_bytes(),
                compress_type=zipfile.ZIP_DEFLATED,
                compresslevel=9,
            )

    run(
        [
            sys.executable,
            str(root / "scripts" / "validate-avr-release-package.py"),
            "--package-dir",
            str(output),
        ],
        root,
    )
    print(f"Release package complete: {output}")
    print(f"{sha256(paths['firmware'])}  {names['firmware']}")
    print(f"{sha256(paths['zip'])}  {names['zip']}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2) from error
