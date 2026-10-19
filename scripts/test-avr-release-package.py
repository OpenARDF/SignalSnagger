#!/usr/bin/env python3
"""Regression tests for rejecting unsafe SignalSnagger release packages."""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def validate(root: Path, package: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(root / "scripts" / "validate-avr-release-package.py"),
            "--package-dir",
            str(package),
        ],
        cwd=root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def require_rejected(
    root: Path,
    clean_package: Path,
    test_root: Path,
    name: str,
    mutate,
    expected: str,
) -> None:
    candidate = test_root / name
    shutil.copytree(clean_package, candidate)
    mutate(candidate)
    result = validate(root, candidate)
    if result.returncode == 0:
        raise RuntimeError(f"validator accepted unsafe package: {name}")
    if expected not in result.stdout:
        raise RuntimeError(
            f"validator rejected {name} for the wrong reason:\n{result.stdout}"
        )
    print(f"PASS rejected {name}: {expected}")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="signalsnagger-release-tests.") as temp:
        test_root = Path(temp)
        clean_parent = test_root / "clean"
        subprocess.run(
            [
                sys.executable,
                str(root / "scripts" / "package-avr-release.py"),
                "--output-root",
                str(clean_parent),
            ],
            cwd=root,
            check=True,
        )
        packages = list(clean_parent.iterdir())
        if len(packages) != 1 or not packages[0].is_dir():
            raise RuntimeError("packager did not create exactly one package directory")
        clean_package = packages[0]
        if validate(root, clean_package).returncode != 0:
            raise RuntimeError("validator rejected the unmodified release package")

        require_rejected(
            root,
            clean_package,
            test_root,
            "unexpected-file",
            lambda package: (package / "unexpected.txt").write_text(
                "not a release payload\n", encoding="utf-8"
            ),
            "package files do not match the contract",
        )

        def remove_readme(package: Path) -> None:
            next(package.glob("README-*.txt")).unlink()

        require_rejected(
            root,
            clean_package,
            test_root,
            "missing-readme",
            remove_readme,
            "package files do not match the contract",
        )

        def corrupt_firmware(package: Path) -> None:
            firmware = next(package.glob("*.hex"))
            firmware.write_bytes(firmware.read_bytes() + b"corruption\n")

        require_rejected(
            root,
            clean_package,
            test_root,
            "corrupt-firmware",
            corrupt_firmware,
            "checksum mismatch",
        )

        def truncate_zip(package: Path) -> None:
            archive = next(package.glob("*.zip"))
            archive.write_bytes(archive.read_bytes()[:32])

        require_rejected(
            root,
            clean_package,
            test_root,
            "corrupt-zip",
            truncate_zip,
            "File is not a zip file",
        )

    print("PASS SignalSnagger release-package negative regression tests")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, StopIteration, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2) from error
