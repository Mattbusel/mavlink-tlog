# The .tlog format

A `.tlog` has no file header. It is a sequence of records:

| Bytes | Content |
|---|---|
| 8 | Receive time, microseconds since the Unix epoch, **big-endian** u64 |
| n | One MAVLink frame, exactly as received |

QGroundControl, Mission Planner and MAVProxy all write this format.
Mission Planner and SITL logs may also contain non-MAVLink records (console
text behind a timestamp); readers skip those.

## MAVLink frames

| | MAVLink 1 | MAVLink 2 |
|---|---|---|
| Start byte | `0xFE` | `0xFD` |
| Header | len, seq, sysid, compid, msgid (1 byte) | len, incompat flags, compat flags, seq, sysid, compid, msgid (3 bytes, little endian) |
| Payload | `len` bytes | `len` bytes, trailing zeros trimmed |
| Checksum | CRC-16/MCRF4XX over header (without start byte) and payload, then the message's CRC_EXTRA byte | same |
| Signature | none | 13 bytes if incompat flag `0x01` is set |

## How damage is handled

At each position the reader tries to read a record. It is accepted when the
frame structure is complete and, depending on `CrcPolicy`:

* `Strict`: the message id is known and the checksum matches,
* `Lenient` (default): the checksum matches for known ids; unknown ids are
  accepted if their timestamp is within `unverified_window_us` (one day) of
  the previous record,
* `Off`: structure only, with the same timestamp window.

Otherwise the reader moves one byte forward and tries again. Skipped bytes,
bad checksums and unknown ids are counted in `ReadStats`.

## Sequence statistics

Per sender (system id, component id), with `step = seq - previous_seq`
(mod 256): 0 is a duplicate, 1 to 128 counts `step - 1` lost frames,
above 128 is out of order (the expected sequence is not moved).
