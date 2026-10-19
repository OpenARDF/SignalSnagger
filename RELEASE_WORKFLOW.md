# SignalSnagger Release Workflow

## Branch and version policy

- `main` contains the deployed 0.38 release baseline and is the active
  integration branch for the current firmware line. Release work is prepared on
  a focused topic branch and reviewed before integration.
- The legacy deployed version string `0.38` is retained while reproducing the
  baseline. New stable versions use `x.y.z`; test builds append letters, such as
  `0.38.1a`, `0.38.1b`, through `0.38.1z`, then `0.38.1aa`.
- Versions whose major field is `0` are development releases, including plain
  numeric versions such as `0.38.1`. They use the risk-based hardware policy
  below rather than the comprehensive `1.0.0`-and-later qualification policy.
- A normal deployment increments the patch field. Starting a new numeric base
  resets the test suffix to `a`; a full release removes the suffix.
- `Software/AVR128DA48/SignalSnagger/defs.h` is the canonical version source.
  Do not tag or publish a release whose visible version differs from its tag or
  asset names.

## Common release gate

1. Begin from a clean, synchronized integration state and announce the branch.
2. Confirm that unrelated KiCad edits are excluded.
3. Complete versioned notes at `release-notes/vX.Y.Z.md`, separating user-visible
   changes from stability/reliability work.
4. Run `just release-preflight`.
5. Require byte-identical repeated ELF and HEX builds, zero unwaived warnings,
   and expected flash/RAM size.
6. Whenever a target is programmed, back up flash, EEPROM, fuses, lock bits,
   and user row. For AVR128DA48 UPDI programming, prefer a device-supported
   isolated flash erase. Never assume that writing an Intel HEX file erases
   flash, and prove by readback that EEPROM and configuration memories are
   unchanged.
7. Freeze the exact clean source commit used for the qualified build.

## Pre-1.0 development releases

- Use a risk-based hardware test plan for every `0.x.y` development release.
  Firmware changes require at least a representative-device boot/version check
  and focused testing of the hardware behavior directly affected by the change.
- Treat changes to the Si5351 quadrature strategy, power sequencing, EEPROM or
  configuration persistence, clocks, interrupts, peripheral drivers, and
  programming workflow as high-risk. Test each affected path on hardware before
  release; a test suffix does not reduce this requirement.
- Perform the remaining applicable smoke checks in `FIRMWARE_SAFETY.md` when
  practical. Scoped omissions are normal for development releases and do not
  require an exceptional-release process, but the reviewed release notes must
  list each omitted check and its reason so the release owner knowingly accepts
  it.
- Do not waive all hardware validation for a firmware-bearing development
  release. Documentation-only or workflow-only releases do not require device
  testing when they cannot alter the built artifact.
- Keep passed, failed, and not-performed checks distinct. A skipped or waived
  check must never be reported as passed.

## 1.0.0 and later releases

- Complete the full applicable hardware smoke test in `FIRMWARE_SAFETY.md` on
  representative hardware, including tuning, sensitivity, gain, display,
  controls, LED behavior, Si5351 quadrature reception, the headphone-removal
  60-second countdown, power-down, and restart.
- A hardware-test omission is exceptional and requires an explicit,
  user-approved waiver recorded in the release notes with its reason and risk.
- Keep passed, failed, and waived checks distinct; never present a waived check
  as passing evidence.

## Publication

- SignalSnagger publishes one AVR128DA48 firmware image and does not share
  SignalSlinger's hardware-specific bootloader/update packages.
- Name the asset `SignalSnagger-vX.Y.Z.hex` and publish its SHA-256 checksum.
- Create an annotated `vX.Y.Z` tag only after explicit authorization. Never move
  a published tag.
- Use `release-notes/vX.Y.Z.md` as the GitHub release body.
- After publication, download into a fresh directory and verify checksum, tag
  target, release body, and embedded/displayed version.
- Commit release evidence after the tag is fixed without moving the tag.

## Intentional differences from SignalSlinger

SignalSnagger has one firmware target and no resident bootloader, serial
updater, fuse-provisioning workflow, or multiple hardware images. Those
SignalSlinger gates apply only if the capabilities are introduced and tested.
