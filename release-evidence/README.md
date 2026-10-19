# Release evidence

Each published release has a checked JSON record named `vX.Y.Z.json`. The
record captures the immutable tag target, public GitHub assets and digests, and
post-publication verification results. Evidence is committed after publication
without moving the release tag.

Hardware checks remain documented in the corresponding checked release notes;
a not-performed check is never converted into passing evidence here.

Run `just release-evidence-check` before committing evidence. The validator
requires the exact two-asset SignalSnagger contract, verifies the checked
release-notes SHA-256 digest and annotated tag target, and rejects any required
publication check recorded as false. `exactReleaseImageProgrammedOnHardware`
may be false only when `hardwareEvidence` explicitly identifies the inherited
candidate evidence.
