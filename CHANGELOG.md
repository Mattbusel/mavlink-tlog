# Changelog

## 0.1.4 (2026-11-22)

New features, all additive; nothing existing changed.

- MAVLink 2 signing: `FrameRef::signature_fields` and
  `FrameRef::verify_signature`, `encode_signed` to write signed frames,
  and `SigningVerifier`, which also rejects replays per (system,
  component, link id) stream. Checked against frames pymavlink 2.4.50
  signed. Built-in SHA-256; still no dependencies and `no_std` (the
  verifier needs `std`).
- `TlogReader::follow` reads a log that is still being written: at the
  end of the input a partly written record is kept rather than counted as
  damage. `finish`, `is_following` and `pending_bytes` go with it.
- `SourceStats::longest_gap_us` and `longest_gap_end_us` (the worst
  dropout per sender) and `SourceStats::rate_total_hz`.
- `mavlink-tlog` command line tool: `stats`, `csv`, `json` (with
  `--follow`), `slice` (cut a time window or sender into a new .tlog) and
  `signing`, with time, sender and message filters.

## 0.1.3 (2026-11-21)

- The message table now has 392 messages, up from 353. It was generated
  from pymavlink 2.4.50, whose `all` dialect leaves out 39 messages that
  MAVLink defines, among them ESC_INFO (290), ESC_STATUS (291),
  PROTOCOL_VERSION (300), EVENT (410), COMMAND_CANCEL (80) and
  LINK_NODE_STATUS (8). Frames of those messages were refused as unknown.
- `tools/gen_table.py` now computes CRC_EXTRA straight from the MAVLink
  XML. Every CRC_EXTRA in the old table matches the new one; eight
  messages gained extension fields, so their maximum length grew.

## 0.1.2 (2026-11-20)

Documentation and metadata only; no change to the library code.

- README opens with the damaged-log result against pymavlink 2.4.50
  (5 invented system-0 messages and 22 dropped frames there; none invented
  and all 22 read here), and the crates.io description says the same.

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
