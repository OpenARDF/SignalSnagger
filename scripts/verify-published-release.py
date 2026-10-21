#!/usr/bin/env python3
"""Freshly download and verify a published SignalSnagger GitHub release."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path


REPOSITORY = "OpenARDF/SignalSnagger"


def fail(message: str) -> None:
    raise RuntimeError(message)


def run(command: list[str], root: Path) -> str:
    result = subprocess.run(
        command,
        cwd=root,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return result.stdout


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", required=True)
    options = parser.parse_args()
    version = options.version
    if not re.fullmatch(r"\d+\.\d+\.\d+", version):
        fail(f"published release version must be x.y.z; found {version}")

    root = Path(__file__).resolve().parents[1]
    tag = f"v{version}"
    notes_path = root / "release-notes" / f"{tag}.md"
    if not notes_path.is_file():
        fail(f"checked release notes are missing: {notes_path}")

    release = json.loads(
        run(
            [
                "gh",
                "release",
                "view",
                tag,
                "--repo",
                REPOSITORY,
                "--json",
                "assets,body,isDraft,isPrerelease,name,publishedAt,tagName,url",
            ],
            root,
        )
    )
    if release.get("tagName") != tag or release.get("name") != f"SignalSnagger {tag}":
        fail("GitHub release tag or title does not match the release version")
    if release.get("isDraft") is not False or release.get("isPrerelease") is not False:
        fail("GitHub release must be published as a non-draft, non-prerelease")
    if release.get("body") != notes_path.read_text(encoding="utf-8"):
        fail("GitHub release body does not exactly match the checked release notes")

    expected_names = {
        f"SignalSnagger-{tag}.hex",
        f"SignalSnagger-{tag}-Release-Files.zip",
    }
    assets = release.get("assets")
    if not isinstance(assets, list) or len(assets) != 2:
        fail("GitHub release must contain exactly two public assets")
    remote_assets: dict[str, dict] = {}
    for asset in assets:
        if not isinstance(asset, dict) or not isinstance(asset.get("name"), str):
            fail("GitHub returned malformed asset metadata")
        remote_assets[asset["name"]] = asset
    if set(remote_assets) != expected_names:
        fail("GitHub asset names do not match the exact two-asset policy")

    remote_tags = run(
        [
            "git",
            "ls-remote",
            "--tags",
            "origin",
            f"refs/tags/{tag}",
            f"refs/tags/{tag}^{{}}",
        ],
        root,
    )
    tag_refs = {
        reference: commit
        for commit, reference in (line.split() for line in remote_tags.splitlines())
    }
    tag_object = tag_refs.get(f"refs/tags/{tag}")
    tag_commit = tag_refs.get(f"refs/tags/{tag}^{{}}")
    if not tag_object or not tag_commit or tag_object == tag_commit:
        fail("remote release tag is missing or is not annotated")

    with tempfile.TemporaryDirectory(prefix=f"signalsnagger-{tag}-verify.") as temp:
        download = Path(temp)
        run(
            [
                "gh",
                "release",
                "download",
                tag,
                "--repo",
                REPOSITORY,
                "--dir",
                str(download),
            ],
            root,
        )
        downloaded = {path.name: path for path in download.iterdir() if path.is_file()}
        if set(downloaded) != expected_names:
            fail("fresh download does not contain the exact two public assets")
        for name, path in downloaded.items():
            metadata = remote_assets[name]
            if metadata.get("size") != path.stat().st_size:
                fail(f"fresh-download size does not match GitHub metadata: {name}")
            digest = metadata.get("digest")
            if digest != f"sha256:{sha256(path)}":
                fail(f"fresh-download digest does not match GitHub metadata: {name}")

        firmware_name = f"SignalSnagger-{tag}.hex"
        zip_name = f"SignalSnagger-{tag}-Release-Files.zip"
        payload_names = {
            firmware_name,
            f"SignalSnagger-Release-Info-{tag}.json",
            f"SignalSnagger-Checksums-{tag}.txt",
            f"README-SignalSnagger-{tag}.txt",
        }
        extracted = download / "extracted"
        extracted.mkdir()
        with zipfile.ZipFile(downloaded[zip_name]) as archive:
            # Inspect the flat, bounded payload before writing any member so a
            # malformed remote archive cannot escape or exhaust the temp area.
            member_info = archive.infolist()
            members = [member.filename for member in member_info]
            if len(members) != len(set(members)) or set(members) != payload_names:
                fail("release ZIP does not contain the exact four-file payload")
            if any(member.file_size > 4 * 1024 * 1024 for member in member_info):
                fail("release ZIP contains an unexpectedly large member")
            for name in members:
                if Path(name).name != name:
                    fail(f"release ZIP contains an unsafe member path: {name}")
                (extracted / name).write_bytes(archive.read(name))
        shutil.copyfile(downloaded[zip_name], extracted / zip_name)
        run(
            [
                sys.executable,
                str(root / "scripts" / "validate-avr-release-package.py"),
                "--package-dir",
                str(extracted),
                "--require-clean",
            ],
            root,
        )
        if downloaded[firmware_name].read_bytes() != (extracted / firmware_name).read_bytes():
            fail("standalone firmware does not match the release ZIP member")
        release_info = json.loads(
            (extracted / f"SignalSnagger-Release-Info-{tag}.json").read_text(
                encoding="utf-8"
            )
        )
        if release_info.get("gitCommit") != tag_commit:
            fail("release package commit does not match the annotated tag target")

    evidence_path = root / "release-evidence" / f"{tag}.json"
    if evidence_path.is_file():
        run(
            [
                sys.executable,
                str(root / "scripts" / "validate-release-evidence.py"),
                "--evidence",
                str(evidence_path),
            ],
            root,
        )
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        if evidence.get("tagCommit") != tag_commit:
            fail("checked release evidence does not match the remote tag target")
        if evidence.get("releaseUrl") != release.get("url"):
            fail("checked release evidence does not match the GitHub release URL")
        if evidence.get("publishedUtc") != release.get("publishedAt"):
            fail("checked release evidence does not match the GitHub publication time")
        evidence_assets = {
            asset["fileName"]: (asset["sizeBytes"], asset["sha256"])
            for asset in evidence["assets"]
        }
        for name, metadata in remote_assets.items():
            if evidence_assets.get(name) != (
                metadata.get("size"),
                str(metadata.get("digest", "")).removeprefix("sha256:"),
            ):
                fail(f"checked release evidence does not match GitHub asset: {name}")

    for name in sorted(expected_names):
        metadata = remote_assets[name]
        print(f"{str(metadata['digest']).removeprefix('sha256:')}  {name}")
    print(f"PASS published SignalSnagger release: {release['url']}")


if __name__ == "__main__":
    try:
        main()
    except (
        json.JSONDecodeError,
        OSError,
        RuntimeError,
        subprocess.CalledProcessError,
        zipfile.BadZipFile,
    ) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2) from error
