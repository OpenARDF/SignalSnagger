# Codex Workflow

## Branch discipline

- Announce the current branch before editing.
- `main` is the deployed, release-ready branch. Development work reaches it
  only through a reviewed collector promotion pull request.
- `Development1` is the firmware and software integration branch. Create
  focused firmware, test, documentation, and general workflow topic branches
  from a clean, synchronized `Development1`, and return them there after their
  applicable gates pass.
- `Development1-KiCad` is the separate KiCad and receiver-simulation
  integration branch. Create schematic, PCB, footprint, symbol, and SPICE-model
  topics from that branch and never merge pending KiCad work into
  `Development1` merely to simplify promotion.
- Start implementation on a focused `codex/<topic>` branch from a clean,
  synchronized collector branch unless the user explicitly requests otherwise.
- Use `just branch-route-check Development1` or `just branch-route-check
  Development1-KiCad` before topic work. Before promotion, use `just
  collector-preflight <collector>` on the collector itself.
- Leave changes uncommitted and unpushed unless the user explicitly authorizes
  those actions. Use reviewed pull requests for normal topic integration.
- Collect all approved firmware/software topics in `Development1`, then use one
  cumulative `Development1` pull request to promote that set to `main`. Collect
  KiCad/simulation topics in `Development1-KiCad` and promote them through a
  separate pull request. Do not combine the two pending change sets.
- After either collector is promoted, synchronize both collectors with the new
  `main` before starting another topic so their common baseline stays current.
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
- Treat the workflow's ordered GitHub release sequence as mandatory. Do not
  omit fresh-download verification or post-tag evidence, and never move a
  published tag to include later evidence.
- Use `just release-published-verify X.Y.Z` after publication and `just
  release-evidence-check` before integrating the evidence record.
- SignalSnagger has one AVR128DA48 target and no checked-in SignalSlinger-style
  bootloader/update bundle. Do not claim dual-target, bootloader,
  interrupted-update, or recovery evidence that does not exist.
