#!/usr/bin/env python3

"""Validate repository-local links in tracked Markdown documents."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote, urlparse


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
LINK_PATTERN = re.compile(r"!?\[[^]]*]\(([^)]+)\)")


def repository_markdown_files() -> list[Path]:
    result = subprocess.run(
        [
            "rg",
            "--files",
            "-g",
            "*.md",
            "-g",
            "!build/**",
            "-g",
            "!**/Debug/**",
            "-g",
            "!**/*-backups/**",
        ],
        cwd=REPOSITORY_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return [REPOSITORY_ROOT / line for line in result.stdout.splitlines() if line]


def main() -> int:
    failures: list[str] = []
    checked_links = 0
    markdown_files = repository_markdown_files()

    for markdown_file in markdown_files:
        relative_file = markdown_file.relative_to(REPOSITORY_ROOT)
        content = markdown_file.read_text(encoding="utf-8")
        for match in LINK_PATTERN.finditer(content):
            target = match.group(1).strip().strip("<>")
            parsed = urlparse(target)
            if parsed.scheme or target.startswith("#"):
                continue
            local_path = unquote(target.split("#", 1)[0])
            if not local_path:
                continue
            checked_links += 1
            if not (markdown_file.parent / local_path).exists():
                failures.append(f"{relative_file}: missing {target}")

    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1

    print(f"Checked {checked_links} local links in {len(markdown_files)} Markdown files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
