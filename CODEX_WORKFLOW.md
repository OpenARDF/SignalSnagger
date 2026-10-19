# Codex Workflow

## Branch discipline

- Announce the current branch before editing.
- `main` is the stable release branch and `Development2` is the active
  integration branch.
- Start implementation on a focused `codex/<topic>` branch from a clean,
  synchronized `Development2` unless the user explicitly requests otherwise.
- Leave changes uncommitted and unpushed unless the user explicitly authorizes
  those actions. Use a reviewed pull request for integration.
- Keep unrelated KiCad changes outside firmware commits. Use `just
  stage-preview` before `just stage-non-kicad` or `just commit-push`.

## Build and verification

- Start with `just --list` and use repository recipes instead of reconstructing
  build commands.
- Run `just test` while developing platform-neutral behavior.
- Run `just check` before handing back any source or workflow change.
- Run `just avr-repeatability` when a firmware change needs deterministic-build
  evidence, and `just release-preflight` for a release candidate.
- Treat all compiler, linter, documentation, policy, and build warnings as
  failures unless a narrow waiver is documented.

## Firmware boundaries

- Put platform-neutral validation and state logic in small modules that can be
  exercised by the host regression suite. Keep register and interrupt code in
  the AVR firmware layer.
- Treat Microchip START output, `Config/`, `utils/`, generated makefiles,
  `SignalSnagger.cppproj`, and component metadata as generated or
  vendor-managed. Make only targeted edits and preserve their license blocks.
- `src/linkbus.cpp` is a dormant legacy import with no matching `linkbus.h` and
  is intentionally excluded from `SignalSnagger.cppproj`. Do not add it to the
  build unless the feature is deliberately restored with tests and its missing
  interface.
- The Si5351 quadrature configuration is an experimentally validated strategy,
  not a sequence derived from the Si5351 documentation. Preserve its feedback
  divider, phase relationship, `P3` values, register-write order, and reset
  behavior unless focused regression tests and representative hardware
  measurements justify a change.
- Explain non-obvious ISR, concurrency, persistence, hardware, and register
  invariants in source comments. Avoid comments that only restate code.

## Release boundary

- Follow [RELEASE_WORKFLOW.md](RELEASE_WORKFLOW.md) for versions, release notes,
  clean-source requirements, deterministic builds, hardware evidence, tags,
  assets, and post-publication verification.
- SignalSnagger currently has one AVR128DA48 target and no checked-in
  SignalSlinger-style bootloader/update bundle. Do not claim or fabricate
  dual-target, bootloader, interrupted-update, or recovery evidence.
