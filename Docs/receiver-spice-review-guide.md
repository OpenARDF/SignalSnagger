# Stage 1 receiver SPICE review guide

This guide is for an independent engineering review of the nominal receiver
models on branch `codex/spice-model-stage1`.  Stage 1 should establish correct
model assignment, pin mapping, and plausible nominal behavior from the RF path
through the audio path.  It must not be treated as a hardware-correlated
receiver-performance prediction.

## Check out the review branch

From a fresh clone, or from an existing clean checkout:

```sh
git fetch origin
git switch --create codex/spice-model-stage1 --track origin/codex/spice-model-stage1
git status --short --branch
git rev-parse HEAD
```

If the local branch already exists, use `git switch codex/spice-model-stage1`
and `git pull --ff-only`.  Record the reviewed commit from `git rev-parse HEAD`
with the findings so the review can be tied to an exact revision.

KiCad 9 with `kicad-cli` is required for the focused check.  ngspice is useful
for independent component-level experiments.  The complete repository gate
also requires the pinned AVR toolchain described in
[`BUILD_ENVIRONMENT.md`](../BUILD_ENVIRONMENT.md).

## Reproduce the automated checks

Run these commands from the repository root:

```sh
just --list
just spice-check
just check
```

`just spice-check` exports the hierarchical schematic as a SPICE netlist and
verifies the repository-local model assignments for 20 receiver-path
references.  The expected result currently also reports 63 unrelated
connector, mechanical, and other non-path placeholders.  Those placeholders
are expected in Stage 1 and mean that the exported full schematic is not yet a
standalone end-to-end simulation deck.

On macOS, the focused check narrowly waives KiCad/Homebrew Fontconfig warnings
during non-rendering export.  Report any other warning or error.  If the AVR
toolchain is unavailable, report `just check` as not performed rather than as a
SPICE failure; `just spice-check` is the blocking focused check for this review.

## Review the implementation

Start with these files:

- [`KiCad/SignalSnagger/spice/README.md`](../KiCad/SignalSnagger/spice/README.md)
  records the source basis and known limitation of every model.
- [`KiCad/SignalSnagger/spice/signalsnagger-stage1.lib`](../KiCad/SignalSnagger/spice/signalsnagger-stage1.lib)
  contains the ngspice subcircuits and their declared pin order.
- [`Docs/receiver-simulation-roadmap.md`](receiver-simulation-roadmap.md)
  defines the boundary between Stage 1 and Stage 2.
- `KiCad/SignalSnagger/SignalSnagger.kicad_sch` is the root schematic; the
  receiver models also touch the `audio.kicad_sch` and `processor.kicad_sch`
  child sheets.

Please review the following questions:

1. Do the `.SUBCKT` pin declarations agree with the manufacturer package pins
   and each KiCad `Sim.Pins` mapping?  A pin-order error is a blocking defect.
2. Are the nominal values and behavioral equations reasonable for the named
   production parts over 3.5 MHz to 3.7 MHz and the resulting baseband/audio
   range?  Flag any value whose source or interpretation is questionable.
3. Are the intentional surrogates clearly identified, and is any model likely
   to create misleading gain, noise, bandwidth, clipping, or switching results?
4. Are any active components in the RF-input-to-headphone-output signal path
   missing?  Power, control, connector, and mechanical parts need models only
   when their behavior materially affects that path.
5. Is REF4132B25, the 2.5 V BOM/MPN variant, the correct reference for the
   mid-supply bias circuit?
6. Does the processor placeholder correctly represent the requested Stage 1
   boundary: sample I/PD2 at 44.1 kHz, quantize to 10 bits, and hold the value on
   DAC0/PD6, with Q, decimation, filtering, and demodulation omitted?
7. Are the Stage 2 priorities sufficient to turn these nominal models into a
   defensible receiver-performance model?

## Suggested independent experiments

Use disposable ngspice test decks that `.include` the Stage 1 library and test
one subcircuit at a time.  Record the ngspice version, model parameters, supply
voltages, source/load impedances, and analysis commands with the results.

- Confirm the Si5351 surrogate produces equal-frequency 3.6 MHz clocks with a
  90-degree phase offset and the expected voltage levels.
- Exercise every PI5C3253 address/enable combination and both DG9422 switch
  states; verify selected-path resistance and off-path isolation/capacitance.
- Check the PWB-4-CLC winding polarity, turns/impedance ratio, DC resistance,
  and sensitivity to `KCOUP`.  Treat its 2 pF parasitics as assumptions until
  they are replaced by an S-parameter fit.
- Bias the transistor model at representative schematic operating points and
  compare DC gain and small-signal response at 3.5, 3.6, and 3.7 MHz with the
  onsemi limits.  Do not infer lot-specific noise or compression performance.
- Verify the REF4132 surrogate produces 2.5 V only under valid input/enable
  conditions and note that startup, noise, and load regulation are simplified.
- Exercise each TS912 channel for DC operating point, closed-loop gain,
  bandwidth, output swing, and capacitive-load sensitivity.  Specifically note
  where the behavioral surrogate diverges from the ST data sheet or official
  PSpice model.
- Sweep the LM386 model through the expected audio band and load.  Check nominal
  gain and pole behavior, but treat output power, distortion, noise, and
  clipping recovery as unvalidated.
- Apply DC ramps and near-Nyquist tones to the AVR passthrough.  Confirm
  44.1 kHz sample-and-hold timing, 10-bit transfer steps, full-scale behavior,
  and DAC output resistance.  Aliasing and quantization are expected; firmware
  filtering is not present.
- Check the nominal 50% potentiometer position, then sweep the `W` parameter
  across the usable range to identify settings that materially change gain or
  stability.

If practical, repeat component-level checks using redistributable manufacturer
models without committing those files.  Note the model version, license, and
ngspice compatibility, and report whether it materially changes the Stage 1
behavior.  Vendor-model redistribution and adoption belong in Stage 2.

## Report findings

Please leave a PR review or comment containing:

- reviewed branch and exact commit SHA;
- KiCad and ngspice versions and operating system;
- commands run, with pass/fail/not-performed results;
- blocking pin-mapping or model-selection defects;
- parameter or behavioral-model improvements, with supporting data-sheet
  section or measurement evidence;
- any proposed additions or reprioritization for Stage 2; and
- test decks, plots, and raw measurement conditions when available.

Classify conclusions as data-sheet-derived, model-derived, measured, or inferred.
That labeling will keep nominal simulation results separate from later
hardware-correlated performance claims.
