# Using mavlink-tlog inside another project

mavlink-tlog reads and writes the `.tlog` container (an 8 byte big-endian
microsecond timestamp before each MAVLink frame) and checks every frame's
checksum. It does not decode message fields and does not open links: it
pairs with whatever MAVLink stack the project already has, such as
rust-mavlink (the `mavlink` crate), `mavio` or generated code of its own.

## Add it

```toml
[dependencies]
mavlink-tlog = "0.1"
```

| Feature | Default | Adds | Extra dependencies |
|---|---|---|---|
| `std` | yes | `TlogReader`, `TlogWriter` over `std::io`, `LinkStats` | none |

The crate has no dependencies. Without `std` you keep `TlogSlice`, frame
parsing, `encode` and the checksum table, with no allocation; CI builds that
for `thumbv7em-none-eabihf` and `wasm32-unknown-unknown`.

## The integration in five lines

Recording a live link in a ground station, router or bridge, and reading the
log back:

```rust
use mavlink_tlog::{TlogReader, TlogWriter};

fn record(tlog: &mut TlogWriter<std::io::BufWriter<std::fs::File>>, frame: &[u8]) -> std::io::Result<()> {
    tlog.write_frame_now(frame) // one call per MAVLink frame received or sent
}

fn replay(path: &str) -> std::io::Result<()> {
    let mut log = TlogReader::new(std::io::BufReader::new(std::fs::File::open(path)?));
    while let Some(rec) = log.next_record()? {
        println!("{} us {:?} from {}/{}", rec.timestamp_us, rec.frame.message_name(), rec.frame.sys_id(), rec.frame.comp_id());
    }
    Ok(())
}
```

`write_frame` takes your own timestamp instead of the system clock. The
files open in QGroundControl, Mission Planner, MAVProxy and pymavlink.

## With rust-mavlink

`rec.frame.as_bytes()` is the complete frame, so rust-mavlink's reader
decodes it directly:

```rust,ignore
use mavlink::dialects::ardupilotmega::MavMessage;

let (header, msg): (mavlink::MavHeader, MavMessage) =
    mavlink::MavlinkReader::new(rec.frame.as_bytes()).read_any_message()?;
```

and a frame rust-mavlink wrote with `mavlink::write_versioned_msg` goes into
`TlogWriter::write_frame` unchanged. Both directions are tested against
rust-mavlink 0.21 in
[`interop/rust-mavlink`](../interop/rust-mavlink/tests/rust_mavlink.rs),
a separate CI job so the crate itself keeps no dependencies and its MSRV.

## The surface to depend on

| Item | What it is |
|---|---|
| `TlogReader`, `Record`, `OwnedRecord`, `ReadOptions`, `ReadStats` | streaming reader over any `Read`, with recovery counters |
| `TlogSlice` | the same over a byte slice or memory map, `no_std`, no allocation |
| `TlogWriter` | writer over any `Write` |
| `FrameRef`, `Header`, `encode`, `message_info`, `crc16`, `CrcPolicy` | one MAVLink 1 or 2 frame, building frames, the `all.xml` checksum table |
| `LinkStats`, `SourceStats` | per-sender frames, loss, duplicates, rates |

Damaged input never panics: a bad record is skipped, counted in
`ReadStats`, and reading continues at the next valid frame.

## Compatibility

- MSRV: Rust 1.70, checked in CI. Edition 2021.
- License: MIT.
- `#![forbid(unsafe_code)]`. CI runs on Linux, Windows and macOS.
- Versioning: while the crate is 0.x, a breaking change bumps the minor
  version (0.1 to 0.2) and is listed in [CHANGELOG.md](../CHANGELOG.md).
- Results are compared with pymavlink 2.4.50 on three public logs in CI
  ([testing.md](testing.md)).

Maintained by Matt Busel ([github.com/Mattbusel](https://github.com/Mattbusel)).
Issues and pull requests:
[github.com/Mattbusel/mavlink-tlog](https://github.com/Mattbusel/mavlink-tlog/issues).
