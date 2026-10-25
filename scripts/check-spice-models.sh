#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
schematic="$repo_root/KiCad/SignalSnagger/SignalSnagger.kicad_sch"
model_library="$repo_root/KiCad/SignalSnagger/spice/signalsnagger-stage1.lib"

if [[ -n "${KICAD_CLI:-}" ]]; then
	kicad_cli=$KICAD_CLI
elif command -v kicad-cli >/dev/null 2>&1; then
	kicad_cli=$(command -v kicad-cli)
elif [[ -x /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli ]]; then
	kicad_cli=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
else
	echo "error: kicad-cli is required (or set KICAD_CLI)" >&2
	exit 1
fi

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/signalsnagger-spice.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT
netlist="$tmp_dir/signalsnagger.cir"
kicad_stderr="$tmp_dir/kicad-stderr.log"
mkdir -p "$tmp_dir/cache"

if ! XDG_CACHE_HOME="$tmp_dir/cache" "$kicad_cli" sch export netlist --format spice -o "$netlist" "$schematic" 2>"$kicad_stderr"; then
	cat "$kicad_stderr" >&2
	exit 1
fi

# KiCad's bundled Fontconfig currently warns while reading newer Homebrew
# configuration keys during a non-rendering netlist export.  Waive only that
# known host diagnostic; any other stderr remains a check failure.
if [[ -s "$kicad_stderr" ]]; then
	if grep -v -E '^Fontconfig warning:' "$kicad_stderr" | grep -q .; then
		cat "$kicad_stderr" >&2
		exit 1
	fi
	echo "note: waived known KiCad/Homebrew Fontconfig warnings during non-rendering export" >&2
fi

expected_models=(
	SS_SMMBT2222ALT1G
	SS_DG9422E
	SS_POT_10K
	SS_POT_100K
	SS_PWB_4_CLC
	SS_PI5C3253
	SS_REF4132B25
	SS_MSS1P4_M3
	SS_TS912_DUAL
	SS_LM386_1
	SS_SI5351_QSD
	SS_AVR128DA48_PASS_44K1
)

for model in "${expected_models[@]}"; do
	if ! grep -q -E "^[.]SUBCKT[[:space:]]+$model([[:space:]]|$)" "$model_library"; then
		echo "error: missing .SUBCKT $model in model library" >&2
		exit 1
	fi
	if ! grep -q -E "[[:space:]]$model([[:space:]]|$)" "$netlist"; then
		echo "error: KiCad netlist does not instantiate $model" >&2
		exit 1
	fi
done

target_refs=(Q101 Q102 Q103 Q104 Q105 T101 U102 U103 U104 U105 U201 U202 U302 U304 U901 U902 RV101 RV302 RV901 D301)
for ref in "${target_refs[@]}"; do
	if grep -q -E "^${ref}[[:space:]]+__${ref}([[:space:]]|$)" "$netlist"; then
		echo "error: unresolved Stage 1 model for $ref" >&2
		exit 1
	fi
done

echo "SPICE model assignments verified for ${#target_refs[@]} receiver-path references."
placeholder_count=$(grep -c -E '__[[:alnum:]_+-]+' "$netlist" || true)
echo "Full-schematic export contains $placeholder_count non-path connector/mechanical placeholders; see the Stage 2 roadmap."
