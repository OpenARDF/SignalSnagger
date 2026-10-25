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
	-I"$repo_root/tests/host/stubs" \
	-I"$firmware_root/include" \
	"$repo_root/tests/host/test_main.cpp" \
	"$repo_root/tests/host/receiver_test_stubs.cpp" \
	"$firmware_root/src/audio_path.cpp" \
	"$firmware_root/src/battery_estimator.cpp" \
	"$firmware_root/src/CircularFloatBuff.cpp" \
	"$firmware_root/src/CircularStringBuff.cpp" \
	"$firmware_root/src/CircularUintBuff.cpp" \
	"$firmware_root/src/Goertzel.cpp" \
	"$firmware_root/src/datetime_utils.cpp" \
	"$firmware_root/src/memory_channels.cpp" \
	"$firmware_root/src/periodic_countdown.cpp" \
	"$firmware_root/src/power_on_clock.cpp" \
	"$firmware_root/src/receiver.cpp" \
	"$firmware_root/src/receiver_frequency.cpp" \
	"$firmware_root/src/rf_gain_audio_cue.cpp" \
	"$firmware_root/src/sense_switches.cpp" \
	"$firmware_root/src/signal_score.cpp" \
	"$firmware_root/src/status_wait.cpp" \
	"$firmware_root/src/text_utils.cpp" \
	-o "$test_binary"

"$test_binary"
