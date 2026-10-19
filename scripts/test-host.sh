#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
firmware_root="$repo_root/Software/AVR128DA48/SignalSnagger"
test_binary="$(mktemp "${TMPDIR:-/tmp}/signalsnagger-host-tests.XXXXXX")"
trap 'rm -f "$test_binary"' EXIT

"${CXX:-c++}" \
	-std=c++11 \
	-Wall \
	-Wextra \
	-Werror \
	-pedantic \
	-I"$firmware_root/include" \
	"$repo_root/tests/host/test_main.cpp" \
	"$firmware_root/src/CircularStringBuff.cpp" \
	"$firmware_root/src/receiver_frequency.cpp" \
	-o "$test_binary"

"$test_binary"
