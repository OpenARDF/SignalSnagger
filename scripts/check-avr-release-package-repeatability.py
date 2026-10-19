#!/usr/bin/env python3
"""Require two independently generated release packages to be byte-identical."""

from __future__ import annotations

import hashlib
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    version_source = root / "Software" / "AVR128DA48" / "SignalSnagger" / "defs.h"
    version_line = next(
        line for line in version_source.read_text(encoding="utf-8").splitlines()
        if line.startswith("#define SW_REVISION ")
    )
    version = version_line.split('"')[1]
    with tempfile.TemporaryDirectory(prefix="signalsnagger-release-repeatability.") as temp:
        temp_root = Path(temp)
        package_dirs: list[Path] = []
        for pass_name in ("first", "second"):
            print(f"SignalSnagger release package repeatability: {pass_name} pass")
            parent = temp_root / pass_name
            subprocess.run(
                [
                    sys.executable,
                    str(root / "scripts" / "package-avr-release.py"),
                    "--output-root",
                    str(parent),
                ],
                cwd=root,
                check=True,
            )
            package_dirs.append(parent / f"SignalSnagger-{version}")

        first_names = sorted(path.name for path in package_dirs[0].iterdir())
        second_names = sorted(path.name for path in package_dirs[1].iterdir())
        if first_names != second_names:
            raise RuntimeError("release package file sets differ between passes")
        for name in first_names:
            first = package_dirs[0] / name
            second = package_dirs[1] / name
            if first.read_bytes() != second.read_bytes():
                raise RuntimeError(f"release package artifact differs between passes: {name}")
            digest = hashlib.sha256(first.read_bytes()).hexdigest()
            print(f"{digest}  {name}")
    print("PASS byte-identical SignalSnagger release packages")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, StopIteration, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2) from error
