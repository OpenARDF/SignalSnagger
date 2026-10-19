# SignalSnagger Release Workflow

## Branch and version policy

- `main` contains stable releases. `Development2` is the active integration
  branch; release work is prepared on a focused topic branch and reviewed
  before integration.
- Stable firmware versions use `x.y.z`. Iterative test builds append letters,
  such as `0.12.0a`, `0.12.0b`, through `0.12.0z`, then `0.12.0aa`.
- A normal deployment increments the patch field. Starting a new numeric base
  resets the test suffix to `a`; a full release removes the suffix.
- `Software/AVR128DA48/SignalSnagger/defs.h` is the canonical version source.
  Do not tag or publish a release whose visible firmware version disagrees with
  its tag or asset names.

## Candidate gate

1. Begin from a clean, synchronized integration state and announce the branch.
2. Confirm that unrelated KiCad edits are excluded.
3. Complete versioned release notes at `release-notes/vX.Y.Z.md`. Describe
   user-visible changes and stability/reliability work separately; do not use a
   generated commit list as the final release body.
4. Run the complete deterministic gate:

   ```sh
   just release-preflight
   ```

5. Review both AVR builds and require byte-identical ELF and HEX artifacts,
   zero warnings, and an expected flash/RAM size.
6. Program representative hardware and verify tuning, receiver sensitivity,
   gain, display, controls, and startup behavior. Specifically verify the
   experimentally established Si5351 quadrature behavior. Record any omitted
   hardware check as an explicit user-approved waiver.
7. Freeze the exact clean source commit used for the qualified build.

## Publication

- SignalSnagger currently publishes a single AVR128DA48 firmware image; it does
  not share SignalSlinger's hardware-specific bootloader and update packages.
- Name the firmware asset `SignalSnagger-vX.Y.Z.hex` and publish its SHA-256
  checksum. Build it from the frozen release commit with the reference inputs in
  [BUILD_ENVIRONMENT.md](BUILD_ENVIRONMENT.md).
- Create an annotated `vX.Y.Z` tag only after explicit authorization. Never
  move a published tag.
- Use the checked `release-notes/vX.Y.Z.md` file as the GitHub release body.
- After publication, download the asset into a fresh directory and verify its
  checksum, tag target, release body, and embedded/displayed firmware version.
- Commit release evidence after the tag is fixed without moving the tag.

## Intentional differences from SignalSlinger

SignalSnagger has one firmware target and currently has no resident bootloader,
serial updater, fuse-provisioning workflow, or multiple hardware images.
SignalSlinger's dual-target package, recovery, and bootloader gates therefore do
not apply unless those capabilities are deliberately introduced and tested.
