# Codex Workflow

## Branch discipline

- Announce the current branch before editing.
- `Development` contains the deployed 0.38 release baseline and is the active
  integration branch for this recovery and hardening effort.
- Start implementation on a focused `codex/<topic>` branch from a clean,
  synchronized `Development` unless the user explicitly requests otherwise.
- Leave changes uncommitted and unpushed unless the user explicitly authorizes
  those actions. Use a reviewed pull request for integration.
- Keep unrelated KiCad changes outside firmware commits. Use `just
  stage-preview` before `just stage-non-kicad` or `just commit-push`.

## Build and verification

- Start with `just --list` and use repository recipes instead of reconstructing
  build commands.
- Add a failing host regression test before changing platform-neutral behavior.
- Run `just test` after each behavior slice and `just check` before handoff.
- Run `just avr-repeatability` for deterministic-build evidence, and
  `just release-preflight` for a release candidate.
- Treat all compiler, linter, documentation, policy, and build warnings as
  failures unless a narrow waiver is documented.

## Firmware boundaries

- Put platform-neutral validation and state logic in small modules exercised by
  the host regression suite. Keep register and interrupt code in the AVR layer.
- Treat Microchip START output, `Config/`, `utils/`, generated makefiles,
  `SignalSnagger.cppproj`, and component metadata as generated or
  vendor-managed. Make only targeted edits and preserve license blocks.
- The orphaned LinkBus import was removed because it had no matching interface
  and was never part of the firmware build. Restore that feature only from
  history as a deliberate, tested implementation.
- The Si5351 quadrature configuration is an experimentally validated strategy,
  not a sequence derived from Si5351 documentation. Preserve its feedback
  divider, phase relationship, `P3` values, register-write order, and reset
  behavior unless focused regression tests and representative hardware
  measurements justify a change.
- Explain non-obvious ISR, concurrency, persistence, hardware, and register
  invariants in source comments. Avoid comments that only restate code.

## Release boundary

- Follow [RELEASE_WORKFLOW.md](RELEASE_WORKFLOW.md) for versions, release notes,
  clean-source requirements, deterministic builds, hardware evidence, tags,
  assets, and post-publication verification.
- SignalSnagger has one AVR128DA48 target and no checked-in SignalSlinger-style
  bootloader/update bundle. Do not claim dual-target, bootloader,
  interrupted-update, or recovery evidence that does not exist.
