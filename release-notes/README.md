# Release notes

Create one checked release body per published version as `vX.Y.Z.md`.

Each file must include:

- user-visible changes;
- stability and reliability changes;
- the exact firmware asset and checksum filenames;
- hardware validation performed, with passed checks kept distinct from failed
  or not-performed checks;
- every omitted hardware check and its reason; scoped omissions are expected
  for pre-1.0 development releases, while `1.0.0`-and-later omissions require an
  explicit user-approved waiver;
- a full-changelog link from the previous release.

The checked file is the authoritative GitHub release body.
