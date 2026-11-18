# Changelog

## 0.1.1 (2026-11-17)

Documentation and tests; no changes to the library code.

- New `docs/integration.md`: features, the API to depend on, MSRV,
  versioning, and how to hand frames to rust-mavlink. Its example runs as
  a doctest.
- New `interop/rust-mavlink` test crate and CI job: every frame read from
  rust-mavlink's test log decodes with rust-mavlink 0.21 with the same
  system, component, sequence and message id, and frames rust-mavlink
  writes (MAVLink 1 and 2) are recorded and read back byte for byte.

## 0.1.0 (2026-11-17)

- First release: `TlogReader`, `TlogSlice`, `TlogWriter`, `LinkStats`,
  frame parsing and encoding with the CRC_EXTRA table of MAVLink `all.xml`.
