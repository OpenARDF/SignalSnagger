#!/usr/bin/env bash

set -euo pipefail

: "${AVR_TOOLCHAIN_ROOT:?Set AVR_TOOLCHAIN_ROOT to the AVR-GCC toolchain directory}"
: "${AVR_DFP_ROOT:?Set AVR_DFP_ROOT to the AVR-Dx_DFP 1.9.103 directory}"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
firmware_root="$repo_root/Software/AVR128DA48/SignalSnagger"
project_file="$firmware_root/SignalSnagger.cppproj"
build_root="${SIGNALSNAGGER_BUILD_ROOT:-$repo_root/build/avr}"
cxx="$AVR_TOOLCHAIN_ROOT/bin/avr-g++"
cc="$AVR_TOOLCHAIN_ROOT/bin/avr-gcc"
objcopy="$AVR_TOOLCHAIN_ROOT/bin/avr-objcopy"
size_tool="$AVR_TOOLCHAIN_ROOT/bin/avr-size"

for required_file in "$cxx" "$cc" "$objcopy" "$size_tool" \
	"$AVR_DFP_ROOT/include/avr/ioavr128da48.h"; do
	if [[ ! -e "$required_file" ]]; then
		printf 'Required AVR build input not found: %s\n' "$required_file" >&2
		exit 1
	fi
done

mkdir -p "$build_root"

# Bash 3.2 ships with macOS, so avoid mapfile/readarray here.
sources=()
while IFS= read -r source; do
	sources+=("$source")
done < <(
	rg --no-filename -o 'Compile Include="[^"]+\.(cpp|S)"' "$project_file" |
		sed -E 's/^Compile Include="//; s/"$//; s|\\|/|g'
)

if [[ ${#sources[@]} -eq 0 ]]; then
	printf 'No firmware sources found in %s\n' "$project_file" >&2
	exit 1
fi

# Keep debug symbols in test images, but optimize the real-time 48 kHz audio
# path enough that hardware testing measures firmware behavior rather than
# debug-build interrupt overhead.
common_flags=(
	-funsigned-char
	-funsigned-bitfields
	-DDEBUG
	-I"$firmware_root/Config"
	-I"$firmware_root/include"
	-I"$firmware_root/utils"
	-I"$firmware_root/utils/assembler"
	-I"$firmware_root"
	-I"$AVR_DFP_ROOT/include"
	-O2
	-g2
	-ffunction-sections
	-fdata-sections
	-fno-threadsafe-statics
	-Wall
	-Wextra
	-Werror
	-mmcu=avr128da48
	-B
	"$AVR_DFP_ROOT/gcc/dev/avr128da48"
)

objects=()
for source in "${sources[@]}"; do
	object="$build_root/${source%.*}.o"
	mkdir -p "$(dirname "$object")"
	if [[ "$source" == *.S ]]; then
		"$cc" -x assembler-with-cpp -c \
			-mmcu=avr128da48 \
			-B "$AVR_DFP_ROOT/gcc/dev/avr128da48" \
			-I"$firmware_root/utils" \
			-I"$AVR_DFP_ROOT/include" \
			"$firmware_root/$source" \
			-o "$object"
	else
		"$cxx" "${common_flags[@]}" -c "$firmware_root/$source" -o "$object"
	fi
	objects+=("$object")
done

elf="$build_root/SignalSnagger.elf"
hex="$build_root/SignalSnagger.hex"
"$cxx" "${objects[@]}" \
	-Wl,-Map="$build_root/SignalSnagger.map" \
	-Wl,--start-group \
	-lm \
	-Wl,--end-group \
	-Wl,--gc-sections \
	-mmcu=avr128da48 \
	-B "$AVR_DFP_ROOT/gcc/dev/avr128da48" \
	-o "$elf"
"$objcopy" -O ihex -R .eeprom -R .fuse -R .lock -R .signature \
	-R .user_signatures "$elf" "$hex"
"$size_tool" "$elf"

read -r text_bytes data_bytes bss_bytes < <("$size_tool" "$elf" | awk 'NR == 2 { print $1, $2, $3 }')
flash_bytes=$((text_bytes + data_bytes))
static_ram_bytes=$((data_bytes + bss_bytes))
max_flash_bytes=120000
max_static_ram_bytes=12288

printf 'Flash budget: %u / %u bytes\n' "$flash_bytes" "$max_flash_bytes"
printf 'Static RAM budget: %u / %u bytes\n' "$static_ram_bytes" "$max_static_ram_bytes"

if ((flash_bytes > max_flash_bytes)); then
	printf 'Firmware exceeds the release flash budget\n' >&2
	exit 1
fi
if ((static_ram_bytes > max_static_ram_bytes)); then
	printf 'Firmware exceeds the release static-RAM budget\n' >&2
	exit 1
fi
