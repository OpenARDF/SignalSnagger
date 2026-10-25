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

for policy_file in .editorconfig .gitattributes BUILD_ENVIRONMENT.md \
	CODEX_WORKFLOW.md RELEASE_WORKFLOW.md \
	scripts/check-branch-workflow.sh \
	scripts/package-avr-release.py \
	scripts/validate-avr-release-package.py \
	scripts/check-avr-release-package-repeatability.py \
	scripts/test-avr-release-package.py \
	scripts/verify-published-release.py \
	scripts/validate-release-evidence.py \
	scripts/test-release-evidence.py \
	release-evidence/README.md; do
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
expect_eol lf Software/AVR128DA48/SignalSnagger/Debug/SignalSnagger.hex

version="$(sed -n 's/^#define SW_REVISION "\([^"]*\)".*/\1/p' "$firmware_root/defs.h")"
version="${version%%$'\n'*}"
if [[ "$version" == "0.38" ]]; then
	printf 'version-policy: %s (deployed legacy baseline)\n' "$version"
elif [[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+([a-z]+)?$ ]]; then
	printf 'version-policy: %s\n' "$version"
else
	fail "SW_REVISION must be legacy 0.38 or x.y.z with optional test suffix; found '$version'"
fi

while IFS= read -r source; do
	relative="src\\$(basename "$source")"
	if ! rg -Fq "Compile Include=\"$relative\"" "$firmware_root/SignalSnagger.cppproj"; then
		fail "firmware project omits source: $relative"
	fi
done < <(printf '%s\n' "$firmware_root"/src/*.cpp)

if ! rg -q '^release-preflight:' "$repo_root/Justfile"; then
	fail "Justfile is missing the release-preflight gate"
fi
if ! rg -q '^release-package:' "$repo_root/Justfile"; then
	fail "Justfile is missing deterministic release-package generation"
fi
if ! rg -q '^release-package-validate package_dir:' "$repo_root/Justfile"; then
	fail "Justfile is missing independent release-package validation"
fi
if ! rg -q '^release-package-repeatability:' "$repo_root/Justfile"; then
	fail "Justfile is missing release-package repeatability validation"
fi
if ! rg -q '^release-package-regression:' "$repo_root/Justfile"; then
	fail "Justfile is missing release-package negative regression tests"
fi
if ! rg -q '^release-published-verify version:' "$repo_root/Justfile"; then
	fail "Justfile is missing published-release verification"
fi
if ! rg -q '^release-evidence-check:' "$repo_root/Justfile"; then
	fail "Justfile is missing durable release-evidence validation"
fi
if ! rg -q '^release-evidence-regression:' "$repo_root/Justfile"; then
	fail "Justfile is missing release-evidence negative regression tests"
fi
if ! rg -q '^branch-route-check collector:' "$repo_root/Justfile"; then
	fail "Justfile is missing collector branch-route validation"
fi
if ! rg -q '^collector-preflight collector:' "$repo_root/Justfile"; then
	fail "Justfile is missing the collector promotion preflight"
fi
if ! rg -q 'Development1-KiCad.*just spice-check' "$repo_root/Justfile"; then
	fail "KiCad collector preflight must run the SPICE model gate"
fi
if ! rg -q 'experimentally validated' "$repo_root/CODEX_WORKFLOW.md"; then
	fail "CODEX_WORKFLOW.md must protect the experimentally validated Si5351 strategy"
fi
if ! rg -q '`Development1` is the firmware and software integration branch' "$repo_root/CODEX_WORKFLOW.md"; then
	fail "CODEX_WORKFLOW.md must identify the firmware/software collector"
fi
if ! rg -q '`Development1-KiCad` is the separate KiCad' "$repo_root/CODEX_WORKFLOW.md"; then
	fail "CODEX_WORKFLOW.md must identify the separate KiCad collector"
fi
if ! rg -q '`Development1` collects firmware, software, tests' "$repo_root/RELEASE_WORKFLOW.md"; then
	fail "RELEASE_WORKFLOW.md must route release work through Development1"
fi
if ! rg -q '`Development1-KiCad` separately collects schematics' "$repo_root/RELEASE_WORKFLOW.md"; then
	fail "RELEASE_WORKFLOW.md must keep KiCad work in its own collector"
fi
if ! rg -q '^## Pre-1\.0 development releases$' "$repo_root/RELEASE_WORKFLOW.md"; then
	fail "RELEASE_WORKFLOW.md must define the pre-1.0 development-release policy"
fi
if ! rg -q '^## 1\.0\.0 and later releases$' "$repo_root/RELEASE_WORKFLOW.md"; then
	fail "RELEASE_WORKFLOW.md must define the 1.0.0-and-later release policy"
fi
if ! rg -q 'immediately preceding suffixed candidate' "$repo_root/RELEASE_WORKFLOW.md"; then
	fail "RELEASE_WORKFLOW.md must define qualification inheritance for version-only pre-1.0 promotion"
fi
if ! rg -q '^## Mandatory GitHub release sequence$' "$repo_root/RELEASE_WORKFLOW.md"; then
	fail "RELEASE_WORKFLOW.md must define the mandatory end-to-end GitHub release sequence"
fi
if ! rg -q '^## Post-publication evidence$' "$repo_root/RELEASE_WORKFLOW.md"; then
	fail "RELEASE_WORKFLOW.md must require durable post-publication evidence"
fi

i2c1_read_wait="$(sed -n '/static uint8_t I2C_1_WaitR/,/Returns how many bytes have been sent/p' "$firmware_root/src/i2c.cpp")"
if [[ "$i2c1_read_wait" == *"g_i2c0_timeout_ticks"* ||
	"$i2c1_read_wait" != *"g_i2c1_timeout_ticks"* ]]; then
	fail "I2C_1_WaitR must use only the I2C1 timeout counter"
fi
if ! rg -q '^#define I2C_SPIN_GUARD' "$firmware_root/src/i2c.cpp"; then
	fail "I2C waits require an interrupt-independent spin guard"
fi

eeprom_update_source="$(sed -n '/void EepromManager::updateEEPROMVar/,/void EepromManager::saveAllEEPROM/p' "$firmware_root/src/eeprommanager.cpp")"
channel_name_update="$(sed -n '/case Channel_Name:/,/case Frequency_Mode:/p' "$firmware_root/src/eeprommanager.cpp")"
if [[ "$channel_name_update" == *"avr_eeprom_write_dword"* ||
	"$channel_name_update" != *"avr_eeprom_write_byte"* ]]; then
	fail "Channel_Name updates must write exactly one EEPROM byte per channel"
fi
if [[ "$eeprom_update_source" != *"avr_eeprom_write_word(I2C_failure_count"* ]]; then
	fail "I2C_failure_count updates must preserve its 16-bit EEPROM field width"
fi
if rg -q 'avr_eeprom_write_dword\((RF_Power|I2C_failure_count)' "$firmware_root/src/eeprommanager.cpp"; then
	fail "16-bit EEPROM fields must not be initialized with 32-bit writes"
fi
if ! rg -q 'const Frequency_Hz f = freq_addr\[cnt\];' "$firmware_root/src/eeprommanager.cpp" ||
	! rg -q 'const uint8_t n = name_addr\[cnt\];' "$firmware_root/src/eeprommanager.cpp"; then
	fail "EEPROM array updates must not read one element beyond the source arrays"
fi
if rg -q '> NUMBER_OF_(FREQUENCY_CHANNELS|BATTERY_CAPACITY_VALUES)' \
	"$firmware_root/main.cpp" "$firmware_root/src/eeprommanager.cpp"; then
	fail "count-valued EEPROM indices must reject the one-past-end sentinel"
fi
if rg -q 'g_active_memory[[:space:]]*=[[:space:]]*nextActiveMemory' "$firmware_root/main.cpp"; then
	fail "memory selection must validate INVALID_CHANNEL before updating the active array index"
fi
if ! rg -Uq 'static_assert\(sizeof\(EE_prom\)[[:space:]]*==[[:space:]]*Cw_Offset_Setting' \
	"$firmware_root/src/eeprommanager.cpp"; then
	fail "EEPROM address constants require compile-time layout verification"
fi
if rg -q 'while\s*\(\s*!ADC0_conversionDone\(\)\s*\)' \
	"$firmware_root/main.cpp" "$firmware_root/src/adc.cpp"; then
	fail "ADC foreground waits require an interrupt-independent timeout"
fi
if ! rg -q '^#define ADC_CONVERSION_SPIN_GUARD' "$firmware_root/src/adc.cpp"; then
	fail "ADC conversion waits require a bounded spin guard"
fi
if ! rg -q '^#define EEPROM_READY_SPIN_GUARD' "$firmware_root/src/eeprommanager.cpp"; then
	fail "EEPROM ready waits require a bounded spin guard"
fi
if rg -q 'while \(NVMCTRL\.STATUS & NVMCTRL_EEBUSY_bm\);' "$firmware_root/src/eeprommanager.cpp"; then
	fail "EEPROM writes must not wait forever for NVM readiness"
fi
eeprom_completion_waits="$(rg -c '\(void\)eepromWaitReady\(\);' "$firmware_root/src/eeprommanager.cpp" || true)"
if [[ "$eeprom_completion_waits" != "4" ]]; then
	fail "each EEPROM write helper must wait for its own operation to complete"
fi
if ! rg -q 'EEPromMgr\.queueTuningPersistence\(' "$firmware_root/main.cpp" ||
	! rg -q 'EEPromMgr\.serviceQueuedWrites\(\);' "$firmware_root/main.cpp"; then
	fail "accepted tuning must use foreground-serviced EEPROM persistence"
fi
deferred_eeprom_service="$(sed -n '/void EepromManager::serviceQueuedWrites/,/void EepromManager::flushQueuedWrites/p' "$firmware_root/src/eeprommanager.cpp")"
if [[ "$deferred_eeprom_service" == *"eepromWaitReady"* ||
	"$deferred_eeprom_service" != *"NVMCTRL_EEBUSY_bm"* ||
	"$deferred_eeprom_service" != *"deferredWriteInProgress_ = true"* ]]; then
	fail "foreground EEPROM service must start one byte without waiting for completion"
fi
if ! rg -q 'sine_period_steps <= MAX_SINE_PERIOD_STEPS' "$firmware_root/main.cpp"; then
	fail "diagnostic sine generation must not write beyond its sample table"
fi
if ! rg -q 'sineIndex >= g_sine_period_steps' "$firmware_root/main.cpp"; then
	fail "diagnostic sine playback must recover when its period is reduced"
fi

git -C "$repo_root" diff --check || failure=1
git -C "$repo_root" diff --cached --check || failure=1

if [[ "$failure" -ne 0 ]]; then
	exit 1
fi

printf 'PASS repository policy\n'
