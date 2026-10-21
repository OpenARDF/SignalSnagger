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
- An unsuffixed pre-1.0 release may inherit hardware evidence from its
  immediately preceding suffixed candidate when the only changed firmware input
  is the canonical version string. The final artifact must pass the common gate,
  its embedded version must be verified statically, and the release notes must
  identify both the inherited evidence and final-image checks not performed.
  Any other firmware-input change invalidates this promotion exception.
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

## Mandatory GitHub release sequence

Every SignalSnagger GitHub release must complete this sequence in order. A
successful upload alone is not a completed release.

1. Prepare the unsuffixed version, release notes, and any release-workflow
   changes on a focused topic branch. Run the applicable gates, review the
   complete diff, merge through a pull request, and synchronize a clean
   `main`.
2. On that exact clean `main` commit, run `just release-preflight`, then `just
   release-package`. Run `just release-package-validate
   build/release-packages/SignalSnagger-X.Y.Z` and require the release-info JSON
   to name `HEAD` with `sourceTreeDirty` set to `false`.
3. Confirm that the firmware checksum, full-flash checksum, embedded version,
   flash/RAM budgets, hardware evidence, and all not-performed or waived checks
   agree with the reviewed release notes.
4. After explicit authorization, create and push an annotated `vX.Y.Z` tag at
   that exact release commit. A published tag is immutable and must never be
   moved to include evidence or later documentation.
5. Create a non-draft, non-prerelease GitHub release named `SignalSnagger
   vX.Y.Z`. Use `release-notes/vX.Y.Z.md` verbatim as its body and publish
   exactly two public assets:
   - `SignalSnagger-vX.Y.Z.hex`
   - `SignalSnagger-vX.Y.Z-Release-Files.zip`
6. Run `just release-published-verify X.Y.Z`. This must freshly download both
   assets, verify GitHub's asset metadata and SHA-256 digests, compare the
   standalone HEX with the ZIP member, validate the ZIP's exact members and
   clean-source metadata, confirm the annotated tag target, compare the release
   body byte-for-byte, and verify the embedded version.
7. Add `release-evidence/vX.Y.Z.json` on a new topic branch, run `just check`,
   merge it through a pull request, and leave the release tag fixed at the
   earlier release commit. Run `just release-published-verify X.Y.Z` again from
   synchronized `main` so the checked evidence is compared with GitHub. Delete
   merged remote topic branches after their state is safely integrated.

SignalSnagger publishes one AVR128DA48 firmware image. The ZIP contains that
same firmware HEX plus release-info JSON, SHA-256 checksums, and a README. It
does not contain SignalSlinger bootloader, updater, first-install, provisioning,
or second-hardware-variant files that SignalSnagger cannot use.

## Post-publication evidence

Post-publication evidence is mandatory. Each `release-evidence/vX.Y.Z.json`
record must identify the immutable tag and peeled commit, GitHub release URL and
publication time, checked release-notes digest, both public asset names, sizes,
and SHA-256 digests, and the fresh-download verification results.

Evidence must keep passed, failed, waived, inherited, and not-performed checks
distinct. In particular, `exactReleaseImageProgrammedOnHardware` must remain
`false` when hardware testing used an immediately preceding suffixed candidate;
the accompanying `hardwareEvidence` text must identify that inheritance. Run
`just release-evidence-check` to validate all checked records and `just
release-evidence-regression` to confirm that invalid or overstated evidence is
rejected.

## Intentional differences from SignalSlinger

SignalSnagger has one firmware target and no resident bootloader, serial
updater, fuse-provisioning workflow, or multiple hardware images. Those
SignalSlinger gates apply only if the capabilities are introduced and tested.
