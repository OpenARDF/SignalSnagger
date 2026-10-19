# SignalSnagger Release Workflow

## Branch and version policy

- `Development` contains the deployed 0.38 release baseline and is the active
  integration branch for the current firmware line. Release work is prepared on
  a focused topic branch and reviewed before integration.
- The legacy deployed version string `0.38` is retained while reproducing the
  baseline. New stable versions use `x.y.z`; test builds append letters, such as
  `0.38.1a`, `0.38.1b`, through `0.38.1z`, then `0.38.1aa`.
- A normal deployment increments the patch field. Starting a new numeric base
  resets the test suffix to `a`; a full release removes the suffix.
- `Software/AVR128DA48/SignalSnagger/defs.h` is the canonical version source.
  Do not tag or publish a release whose visible version differs from its tag or
  asset names.

## Candidate gate

1. Begin from a clean, synchronized integration state and announce the branch.
2. Confirm that unrelated KiCad edits are excluded.
3. Complete versioned notes at `release-notes/vX.Y.Z.md`, separating user-visible
   changes from stability/reliability work.
4. Run `just release-preflight`.
5. Require byte-identical repeated ELF and HEX builds, zero unwaived warnings,
   and expected flash/RAM size.
6. Before programming, back up flash, EEPROM, fuses, lock bits, and user row.
   For AVR128DA48 UPDI programming, prefer a device-supported isolated flash
   erase. Never assume that writing an Intel HEX file erases flash, and prove by
   readback that EEPROM and configuration memories are unchanged.
7. Program representative hardware and verify tuning, sensitivity, gain,
   display, controls, LED behavior, the headphone-removal 60-second countdown,
   and power-down. Specifically verify the experimentally established Si5351
   quadrature behavior. Record omissions as explicit user-approved waivers.
8. Freeze the exact clean source commit used for the qualified build.

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
