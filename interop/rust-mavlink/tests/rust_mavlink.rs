//! mavlink-tlog hands frames to rust-mavlink, and records what rust-mavlink
//! writes, with nothing lost or changed in between.

use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink::{MavHeader, MavlinkVersion, Message};
use mavlink_tlog::{TlogReader, TlogSlice, TlogWriter};

const LOG: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../tests/data/rust-mavlink-log.tlog"
);

/// Every frame mavlink-tlog reads from rust-mavlink's own test log decodes
/// with rust-mavlink, with the same header and message id.
#[test]
fn tlog_frames_decode_with_rust_mavlink() {
    let bytes = std::fs::read(LOG).unwrap();
    let mut log = TlogSlice::new(&bytes);
    let mut decoded = 0;
    for rec in log.by_ref() {
        let frame = rec.frame.as_bytes();
        let mut reader = mavlink::MavlinkReader::new(frame);
        let (header, msg): (MavHeader, MavMessage) =
            reader.read_any_message().unwrap_or_else(|e| {
                panic!(
                    "rust-mavlink rejected a frame at {} us: {e:?}",
                    rec.timestamp_us
                )
            });
        assert_eq!(header.system_id, u32::from(rec.frame.sys_id()));
        assert_eq!(header.component_id, rec.frame.comp_id());
        assert_eq!(header.sequence, rec.frame.seq());
        assert_eq!(msg.message_id(), rec.frame.msg_id());
        decoded += 1;
    }
    assert_eq!(log.stats().skipped_bytes, 0);
    assert_eq!(
        decoded, 1426,
        "frame count pinned by the pymavlink reference test"
    );
}

/// Messages written by rust-mavlink and recorded with TlogWriter read back
/// byte for byte, and decode to the same message.
#[test]
fn rust_mavlink_frames_round_trip_through_a_tlog() {
    let msgs = [
        MavMessage::HEARTBEAT(mavlink::dialects::ardupilotmega::HEARTBEAT_DATA::default()),
        MavMessage::GLOBAL_POSITION_INT(
            mavlink::dialects::ardupilotmega::GLOBAL_POSITION_INT_DATA {
                lat: 473_977_418,
                lon: 85_455_939,
                alt: 488_000,
                ..Default::default()
            },
        ),
    ];
    let mut tlog = TlogWriter::new(Vec::new());
    let mut written = Vec::new();
    for (i, msg) in msgs.iter().enumerate() {
        for version in [MavlinkVersion::V1, MavlinkVersion::V2] {
            let header = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: i as u8,
            };
            let mut frame = Vec::new();
            mavlink::write_versioned_msg(&mut frame, version, header, msg).unwrap();
            tlog.write_frame(1_700_000_000_000_000 + i as u64, &frame)
                .unwrap();
            written.push(frame);
        }
    }
    let file = tlog.into_inner();

    let mut reader = TlogReader::new(file.as_slice());
    let mut n = 0;
    while let Some(rec) = reader.next_record().unwrap() {
        assert_eq!(rec.frame.as_bytes(), written[n].as_slice());
        let (_, msg): (MavHeader, MavMessage) = mavlink::MavlinkReader::new(rec.frame.as_bytes())
            .read_any_message()
            .unwrap();
        assert_eq!(msg, msgs[n / 2]);
        n += 1;
    }
    assert_eq!(n, written.len());
    assert_eq!(reader.stats().bad_crc, 0);
}
