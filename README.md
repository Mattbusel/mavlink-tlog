# mavlink-tlog: a damaged flight log is still evidence

[![crates.io](https://img.shields.io/crates/v/mavlink-tlog.svg)](https://crates.io/crates/mavlink-tlog)
[![docs.rs](https://docs.rs/mavlink-tlog/badge.svg)](https://docs.rs/mavlink-tlog)
[![CI](https://github.com/Mattbusel/mavlink-tlog/actions/workflows/ci.yml/badge.svg)](https://github.com/Mattbusel/mavlink-tlog/actions/workflows/ci.yml)

**Your telemetry tools make up messages when a log is damaged, and you
can't tell which ones.** A reader that does not check each frame against its
message definition turns broken bytes into data. On a public SITL log with
cut-off frame headers, the reference Python reader (pymavlink 2.4.50)
reports 5 messages from a system 0 that never existed (`UNKNOWN_658xx`)
and drops the 22 real frames around the damage. That is fiction in a
flight record, and it is the record you open after something went wrong. This crate checks every frame's checksum against all 353
messages in MAVLink's `all.xml`, refuses the broken bytes, counts them, and
reads the 22 real frames:

| `autotest.tlog` (35,750 frames, console text mixed in) | pymavlink 2.4.50 | mavlink-tlog |
|---|---|---|
| Messages invented from damaged bytes | 5 (from system 0) | **0** |
| Real frames next to the damage | 22 dropped | **22 read** |
| Bytes it could not use | not reported | counted in `stats()` |
| Undamaged logs (1,426 and 163,851 frames) | reference | identical on every count |

If you investigate incidents, sign off flights or train models on
telemetry, a reader that invents a sender is worse than one that stops.
The pinned test: [How it is tested](#how-it-is-tested).

Read and write MAVLink `.tlog` telemetry logs in Rust: the files
QGroundControl, Mission Planner and MAVProxy save from a drone's telemetry
link (each MAVLink frame behind an 8 byte big-endian microsecond timestamp).

* **Reads damaged logs.** Every frame's checksum is verified with the
  CRC_EXTRA of all 353 messages in MAVLink's `all.xml` (built in, no dialect
  crate needed). A damaged record is skipped and reading continues at the
  next valid one; what was skipped is counted.
* **Streams or borrows.** `TlogReader` reads any `std::io::Read` with a
  64 KiB buffer; `TlogSlice` walks a byte slice (or a memory map) without
  allocating and works in `no_std`.
* **Writes logs.** `TlogWriter` produces files QGroundControl and pymavlink
  open, and `encode` builds MAVLink 1 and 2 frames with correct checksums.
* **Measures the link.** `LinkStats` gives frames, sequence-gap loss,
  duplicates, out of order frames and per-message rates for every sender
  (system id, component id).
* Pure Rust, no dependencies, `#![forbid(unsafe_code)]`.

It works on raw frames, so it pairs with any decoder: pass
`record.frame.as_bytes()` to [`mavlink`](https://crates.io/crates/mavlink)
(rust-mavlink), `mavio`, or your own code.

## Use

```toml
[dependencies]
mavlink-tlog = "0.1"
```

```rust
use mavlink_tlog::{LinkStats, TlogReader};

let file = std::fs::File::open("tests/data/rust-mavlink-log.tlog")?;
let mut log = TlogReader::new(std::io::BufReader::new(file));
let mut stats = LinkStats::default();
while let Some(rec) = log.next_record()? {
    stats.observe(rec.timestamp_us, &rec.frame);
    if rec.frame.message_name() == Some("HEARTBEAT") {
        // rec.frame.payload() is the raw payload; rec.frame.as_bytes() the whole frame.
    }
}
for ((sys, comp), s) in stats.sources() {
    println!("{sys}/{comp}: {} frames, {:.2}% lost", s.received, s.loss_ratio() * 100.0);
}
println!("{:?}", log.stats()); // records, skipped bytes, bad checksums
# Ok::<(), std::io::Error>(())
```

Recording a live link (for example inside a ground station or router):

```rust
use mavlink_tlog::TlogWriter;

let mut tlog = TlogWriter::new(Vec::new()); // or BufWriter<File>
let frame = [0xFE, 9, 7, 1, 1, 0, 4, 0, 0, 0, 2, 3, 0x51, 3, 3, 0x63, 0x91];
tlog.write_frame_now(&frame)?;
# Ok::<(), std::io::Error>(())
```

`examples/tlog_stats.rs` prints a summary of a log. On the 40 minute
ArduPilot flight log from pymavlink's test data:

```text
$ cargo run --release --example tlog_stats -- flight.tlog
flight.tlog: 163851 frames, 23 bytes skipped, 0 bad checksums

sender 1/1: 156418 frames over 2443.8 s, 2847 lost (1.79%), 4 duplicates, 294 out of order
  SYS_STATUS                   6364  2.6 Hz
  ...
sender 51/68: 4820 frames over 2443.4 s, 21 lost (0.43%), 2358 duplicates, 106 out of order
  RADIO_STATUS                 2410  1.0 Hz
  RADIO                        2410  1.0 Hz
```

## How it is tested

The tests compare against **pymavlink** (2.4.50, `all` dialect), the
reference Python implementation ArduPilot uses, on three public logs:

| Log | Size | Result |
|---|---|---|
| rust-mavlink `tests/log.tlog` | 1,426 frames | identical: frame count, count per message id, per sender, first and last timestamp, loss, duplicates, out of order |
| pymavlink `flight.tlog`, 40 min ArduPilot flight | 163,851 frames | identical on every one of those numbers |
| MissionPlannerRust `autotest.tlog`, SITL with console text mixed in | 35,750 frames | see below |

On the third log the two disagree, and the difference is explained in the
test: the file has 5 places where a frame header was cut off and a whole
frame follows (`fd 09 00`, then `fd 09 00 00 ...`). pymavlink parses each of
those as a bogus message from system 0 (`UNKNOWN_658xx`) and drops the 22
real frames around the damage; this crate skips the broken bytes and reads
those frames. The test pins both numbers: pymavlink's count minus the 5
bogus messages plus the 22 recovered frames, per message id.

Also tested: pymavlink-encoded frames parse and re-encode byte for byte;
rewriting a clean log reproduces it byte for byte; a flipped byte loses only
its own record; injected garbage is skipped exactly; the streaming reader
gives the same records and counters as the slice reader with reads of 1, 3,
64 and 100,000 bytes; random input never panics; rust-mavlink 0.21 decodes
every frame read from its own test log with the same header, and frames it
writes come back byte for byte (`interop/rust-mavlink`). CI regenerates the pymavlink
numbers and the message table and fails if they drift.

See [docs/format.md](docs/format.md) for the file format,
[docs/testing.md](docs/testing.md) for how to run the full suite, and
[docs/integration.md](docs/integration.md) for using it inside another
project (features, the API to depend on, MSRV, versioning).

## License

MIT. See [tests/data/README.md](tests/data/README.md) for the test logs'
sources and licenses (the two large ones are downloaded, not redistributed).
