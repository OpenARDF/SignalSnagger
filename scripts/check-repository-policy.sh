#!/usr/bin/env bash

set -euo pipefail
export LC_ALL=C

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
firmware_root="$repo_root/Software/AVR128DA48/SignalSnagger"
failure=0

fail() {
	printf 'ERROR: %s\n' "$1" >&2
	failure=1
}

expect_file() {
	if [[ -f "$repo_root/$1" ]]; then
		printf 'present: %s\n' "$1"
	else
		fail "required policy file is missing: $1"
	fi
}

expect_ignored() {
	if git -C "$repo_root" check-ignore -q --no-index "$1"; then
		printf 'ignored: %s\n' "$1"
	else
		fail "generated or temporary path is visible: $1"
	fi
}

expect_eol() {
	local expected="$1"
	local path="$2"
	local actual
	actual="$(git -C "$repo_root" check-attr eol -- "$path" | sed 's/^.*: eol: //')"
	if [[ "$actual" == "$expected" ]]; then
		printf 'eol=%s: %s\n' "$expected" "$path"
	else
		fail "$path has eol=$actual, expected $expected"
	fi
}

for policy_file in .editorconfig .gitattributes BUILD_ENVIRONMENT.md CODEX_WORKFLOW.md RELEASE_WORKFLOW.md; do
	expect_file "$policy_file"
done

expect_ignored "build/example.hex"
expect_ignored "Software/AVR128DA48/.vs/example.suo"
expect_ignored "Software/AVR128DA48/SignalSnagger/Debug/example.o"
expect_ignored "KiCad/SignalSnagger/~SignalSnagger.kicad_sch.lck"

expect_eol lf README.md
expect_eol lf Justfile
expect_eol lf scripts/avr-build.sh
expect_eol lf Software/AVR128DA48/SignalSnagger/main.cpp
expect_eol lf KiCad/SignalSnagger/SignalSnagger.kicad_sch
expect_eol crlf Software/AVR128DA48/SignalSnagger/SignalSnagger.cppproj

version="$(sed -n 's/^#define SW_REVISION "\([^"]*\)".*/\1/p' "$firmware_root/defs.h")"
version="${version%%$'\n'*}"
if [[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+([a-z]+)?$ ]]; then
	printf 'version-policy: %s\n' "$version"
else
	fail "SW_REVISION must use x.y.z or letter-suffixed test form, found '$version'"
fi

while IFS= read -r source; do
	if [[ "$(basename "$source")" == "linkbus.cpp" ]]; then
		continue
	fi
	relative="src\\$(basename "$source")"
	if ! rg -Fq "Compile Include=\"$relative\"" "$firmware_root/SignalSnagger.cppproj"; then
		fail "firmware project omits source: $relative"
	fi
done < <(printf '%s\n' "$firmware_root"/src/*.cpp)

if ! rg -q '`src/linkbus\.cpp` is a dormant legacy import' "$repo_root/CODEX_WORKFLOW.md"; then
	fail "the intentionally excluded legacy linkbus source must remain documented"
fi

if ! rg -q '^release-preflight:' "$repo_root/Justfile"; then
	fail "Justfile is missing the release-preflight gate"
fi

if ! rg -q 'experimentally validated' "$repo_root/CODEX_WORKFLOW.md" 2>/dev/null; then
	fail "CODEX_WORKFLOW.md must protect the experimentally validated Si5351 strategy"
fi

git -C "$repo_root" diff --check || failure=1
git -C "$repo_root" diff --cached --check || failure=1

if [[ "$failure" -ne 0 ]]; then
	exit 1
fi

printf 'PASS repository policy\n'
