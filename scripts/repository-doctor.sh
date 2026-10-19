#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
missing=0

printf 'Repository: %s\n' "$repo_root"
printf 'Branch: %s\n' "$(git -C "$repo_root" branch --show-current)"

for tool in bash git rg just gitleaks shellcheck shfmt python3 c++ cmp shasum; do
	if command -v "$tool" >/dev/null 2>&1; then
		printf '%-12s available\n' "$tool"
	else
		printf '%-12s MISSING\n' "$tool" >&2
		missing=1
	fi
done

if [[ -z "${AVR_TOOLCHAIN_ROOT:-}" ]]; then
	printf 'AVR_TOOLCHAIN_ROOT is not set\n' >&2
	missing=1
elif [[ ! -x "$AVR_TOOLCHAIN_ROOT/bin/avr-g++" ]]; then
	printf 'AVR compiler is missing: %s/bin/avr-g++\n' "$AVR_TOOLCHAIN_ROOT" >&2
	missing=1
else
	compiler_output="$("$AVR_TOOLCHAIN_ROOT/bin/avr-g++" --version)"
	compiler_version="${compiler_output%%$'\n'*}"
	printf 'AVR compiler: %s\n' "$compiler_version"
	if [[ "$compiler_version" != *"3.7.0_518"* || "$compiler_version" != *"7.3.0"* ]]; then
		printf 'Reference AVR compiler is 3.7.0.518 / GCC 7.3.0\n' >&2
		missing=1
	fi
fi

if [[ -z "${AVR_DFP_ROOT:-}" ]]; then
	printf 'AVR_DFP_ROOT is not set\n' >&2
	missing=1
elif [[ ! -f "$AVR_DFP_ROOT/Atmel.AVR-Dx_DFP.pdsc" ||
	! -f "$AVR_DFP_ROOT/include/avr/ioavr128da48.h" ]]; then
	printf 'AVR-Dx device pack is incomplete: %s\n' "$AVR_DFP_ROOT" >&2
	missing=1
elif ! rg -q '<release version="1\.9\.103"' "$AVR_DFP_ROOT/Atmel.AVR-Dx_DFP.pdsc"; then
	printf 'Reference AVR-Dx_DFP version is 1.9.103\n' >&2
	missing=1
else
	printf 'AVR device pack: 1.9.103\n'
fi

if [[ "$missing" -ne 0 ]]; then
	exit 1
fi

printf 'PASS repository doctor\n'
