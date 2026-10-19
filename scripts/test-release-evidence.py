#!/usr/bin/env python3
"""Regression tests for rejecting invalid SignalSnagger release evidence."""

from __future__ import annotations

import copy
import json
import subprocess
import sys
import tempfile
from pathlib import Path


def validate(root: Path, evidence: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(root / "scripts" / "validate-release-evidence.py"),
            "--evidence",
            str(evidence),
        ],
        cwd=root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def require_rejected(
    root: Path,
    original: dict,
    temp_root: Path,
    name: str,
    mutate,
    expected: str,
) -> None:
    candidate = copy.deepcopy(original)
    mutate(candidate)
    candidate_path = temp_root / "v0.38.1.json"
    candidate_path.write_text(f"{json.dumps(candidate, indent=2)}\n", encoding="utf-8")
    result = validate(root, candidate_path)
    if result.returncode == 0:
        raise RuntimeError(f"validator accepted invalid release evidence: {name}")
    if expected not in result.stdout:
        raise RuntimeError(
            f"validator rejected {name} for the wrong reason:\n{result.stdout}"
        )
    print(f"PASS rejected {name}: {expected}")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    source = root / "release-evidence" / "v0.38.1.json"
    original = json.loads(source.read_text(encoding="utf-8"))
    valid = validate(root, source)
    if valid.returncode != 0:
        raise RuntimeError(f"validator rejected checked evidence:\n{valid.stdout}")

    with tempfile.TemporaryDirectory(prefix="signalsnagger-evidence-tests.") as temp:
        temp_root = Path(temp)
        require_rejected(
            root,
            original,
            temp_root,
            "wrong-tag-target",
            lambda evidence: evidence.__setitem__("tagCommit", "0" * 40),
            "tagCommit does not match",
        )
        require_rejected(
            root,
            original,
            temp_root,
            "missing-asset",
            lambda evidence: evidence["assets"].pop(),
            "exactly two assets",
        )
        require_rejected(
            root,
            original,
            temp_root,
            "false-download-check",
            lambda evidence: evidence["verification"].__setitem__(
                "freshDownloadChecksumsMatch", False
            ),
            "freshDownloadChecksumsMatch must be true",
        )
        require_rejected(
            root,
            original,
            temp_root,
            "wrong-notes-digest",
            lambda evidence: evidence["releaseNotes"].__setitem__(
                "sha256", "0" * 64
            ),
            "release-notes SHA-256 does not match",
        )
        require_rejected(
            root,
            original,
            temp_root,
            "unexpected-field",
            lambda evidence: evidence.__setitem__("unsupported", True),
            "fields do not match the policy",
        )
        require_rejected(
            root,
            original,
            temp_root,
            "missing-inheritance-explanation",
            lambda evidence: evidence["verification"].__setitem__(
                "hardwareEvidence", "The exact release image was not programmed."
            ),
            "must identify inherited hardware evidence",
        )

    print("PASS SignalSnagger release-evidence negative regression tests")


if __name__ == "__main__":
    try:
        main()
    except (json.JSONDecodeError, OSError, RuntimeError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2) from error
