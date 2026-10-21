#!/usr/bin/env python3
"""Validate durable SignalSnagger GitHub release evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path


ROOT_KEYS = {
    "format",
    "version",
    "tag",
    "tagCommit",
    "releaseUrl",
    "publishedUtc",
    "verifiedUtc",
    "releaseNotes",
    "assets",
    "verification",
}
NOTES_KEYS = {"fileName", "sha256", "matchesGitHubBody"}
ASSET_KEYS = {"fileName", "sizeBytes", "sha256"}
VERIFICATION_KEYS = {
    "annotatedTag",
    "tagTargetMatchesPackageCommit",
    "packageBuiltFromCleanTree",
    "freshDownloadHasExactAssetSet",
    "freshDownloadChecksumsMatch",
    "zipContentsAndBytesMatch",
    "embeddedVersionMatchesTag",
    "exactReleaseImageProgrammedOnHardware",
    "hardwareEvidence",
}
REQUIRED_TRUE_CHECKS = VERIFICATION_KEYS - {
    "exactReleaseImageProgrammedOnHardware",
    "hardwareEvidence",
}


def fail(message: str) -> None:
    raise RuntimeError(message)


def require_exact_keys(value: object, expected: set[str], description: str) -> dict:
    if not isinstance(value, dict):
        fail(f"{description} must be an object")
    actual = set(value)
    if actual != expected:
        fail(
            f"{description} fields do not match the policy: "
            f"missing={sorted(expected - actual)}, unexpected={sorted(actual - expected)}"
        )
    return value


def require_string(value: object, description: str) -> str:
    if not isinstance(value, str) or not value:
        fail(f"{description} must be a non-empty string")
    return value


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(root: Path, *arguments: str) -> str:
    result = subprocess.run(
        ["git", *arguments],
        cwd=root,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout.strip()


def validate_timestamp(value: object, description: str) -> datetime:
    text = require_string(value, description)
    if not re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z", text):
        fail(f"{description} must be an ISO-8601 UTC timestamp")
    return datetime.fromisoformat(text.replace("Z", "+00:00"))


def validate_evidence(root: Path, evidence_path: Path) -> None:
    evidence = require_exact_keys(
        json.loads(evidence_path.read_text(encoding="utf-8")),
        ROOT_KEYS,
        "release evidence",
    )
    if evidence.get("format") != "signalsnagger-release-evidence-v1":
        fail("unsupported release-evidence format")

    version = require_string(evidence.get("version"), "version")
    if not re.fullmatch(r"\d+\.\d+\.\d+", version):
        fail(f"release evidence version is invalid: {version}")
    tag = require_string(evidence.get("tag"), "tag")
    if tag != f"v{version}":
        fail("release evidence tag does not match its version")
    expected_name = f"v{version}.json"
    if evidence_path.name != expected_name:
        fail(f"release evidence file must be named {expected_name}")

    tag_commit = require_string(evidence.get("tagCommit"), "tagCommit")
    if not re.fullmatch(r"[0-9a-f]{40}", tag_commit):
        fail("tagCommit must be a full lowercase Git commit ID")
    if git(root, "cat-file", "-t", tag) != "tag":
        fail(f"{tag} is not an annotated tag")
    if git(root, "rev-parse", f"{tag}^{{}}") != tag_commit:
        fail("tagCommit does not match the annotated tag target")

    expected_url = f"https://github.com/OpenARDF/SignalSnagger/releases/tag/{tag}"
    if evidence.get("releaseUrl") != expected_url:
        fail("releaseUrl does not match the SignalSnagger GitHub release")
    published = validate_timestamp(evidence.get("publishedUtc"), "publishedUtc")
    verified = validate_timestamp(evidence.get("verifiedUtc"), "verifiedUtc")
    if verified < published:
        fail("verifiedUtc cannot precede publishedUtc")

    notes = require_exact_keys(evidence.get("releaseNotes"), NOTES_KEYS, "releaseNotes")
    expected_notes_name = f"release-notes/{tag}.md"
    if notes.get("fileName") != expected_notes_name:
        fail(f"releaseNotes.fileName must be {expected_notes_name}")
    notes_path = root / expected_notes_name
    if not notes_path.is_file():
        fail(f"checked release notes are missing: {expected_notes_name}")
    if notes.get("sha256") != sha256(notes_path):
        fail("release-notes SHA-256 does not match the checked file")
    if notes.get("matchesGitHubBody") is not True:
        fail("matchesGitHubBody must be true")

    assets = evidence.get("assets")
    if not isinstance(assets, list) or len(assets) != 2:
        fail("release evidence must contain exactly two assets")
    expected_assets = {
        f"SignalSnagger-{tag}.hex",
        f"SignalSnagger-{tag}-Release-Files.zip",
    }
    actual_assets: set[str] = set()
    for index, raw_asset in enumerate(assets):
        asset = require_exact_keys(raw_asset, ASSET_KEYS, f"assets[{index}]")
        name = require_string(asset.get("fileName"), f"assets[{index}].fileName")
        actual_assets.add(name)
        size = asset.get("sizeBytes")
        if not isinstance(size, int) or isinstance(size, bool) or size <= 0:
            fail(f"assets[{index}].sizeBytes must be a positive integer")
        digest = require_string(asset.get("sha256"), f"assets[{index}].sha256")
        if not re.fullmatch(r"[0-9a-f]{64}", digest):
            fail(f"assets[{index}].sha256 must be a lowercase SHA-256 digest")
    if actual_assets != expected_assets:
        fail("release evidence asset names do not match the exact two-asset policy")

    verification = require_exact_keys(
        evidence.get("verification"),
        VERIFICATION_KEYS,
        "verification",
    )
    for check in sorted(REQUIRED_TRUE_CHECKS):
        if verification.get(check) is not True:
            fail(f"verification.{check} must be true")
    programmed = verification.get("exactReleaseImageProgrammedOnHardware")
    if not isinstance(programmed, bool):
        fail("verification.exactReleaseImageProgrammedOnHardware must be Boolean")
    hardware_evidence = require_string(
        verification.get("hardwareEvidence"),
        "verification.hardwareEvidence",
    )
    # An unflashed final image is allowed only by the documented pre-1.0
    # version-only promotion rule, which inherits evidence from its candidate.
    if not programmed and "inherit" not in hardware_evidence.lower():
        fail("unprogrammed exact release images must identify inherited hardware evidence")

    print(f"PASS release evidence: {evidence_path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--evidence",
        action="append",
        type=Path,
        help="Evidence file to validate; defaults to every checked release record",
    )
    options = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    paths = options.evidence or sorted((root / "release-evidence").glob("v*.json"))
    if not paths:
        fail("no release-evidence records found")
    for path in paths:
        validate_evidence(root, path.resolve())


if __name__ == "__main__":
    try:
        main()
    except (json.JSONDecodeError, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2) from error
