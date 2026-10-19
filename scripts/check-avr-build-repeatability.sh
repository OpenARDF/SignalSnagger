#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repeatability_root="$(mktemp -d "${TMPDIR:-/tmp}/signalsnagger-repeatability.XXXXXX")"
first_build="$repeatability_root/first"
second_build="$repeatability_root/second"

cleanup() {
	rm -rf "$repeatability_root"
}
trap cleanup EXIT

printf 'SignalSnagger AVR repeatability: pass 1 of 2\n'
SIGNALSNAGGER_BUILD_ROOT="$first_build" "$repo_root/scripts/avr-build.sh"
printf 'SignalSnagger AVR repeatability: pass 2 of 2\n'
SIGNALSNAGGER_BUILD_ROOT="$second_build" "$repo_root/scripts/avr-build.sh"

for artifact in SignalSnagger.elf SignalSnagger.hex; do
	if ! cmp -s "$first_build/$artifact" "$second_build/$artifact"; then
		printf 'Build artifact differs between passes: %s\n' "$artifact" >&2
		exit 1
	fi
	printf '%s  %s\n' "$(shasum -a 256 "$second_build/$artifact" | cut -d ' ' -f 1)" "$artifact"
done

printf 'PASS byte-identical AVR build artifacts\n'
