//! The flight report and tamper verdict: on the real shipped log, and on
//! copies of it with a known, documented modification.
//!
//! The clean input is `tests/data/rust-mavlink-log.tlog`, 1,426 frames from
//! mavlink/rust-mavlink (MIT OR Apache-2.0, see tests/data/README.md). Each
//! tamper case takes that real log, makes one precise change, and the test
//! says exactly what the change is, so the detection is checked against
//! ground truth rather than invented data.

use mavlink_tlog::*;

fn sample() -> Vec<u8> {
    std::fs::read(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/data/rust-mavlink-log.tlog"
    ))
    .unwrap()
}

/// Every (timestamp, frame bytes) of the clean log, in order.
fn records(bytes: &[u8]) -> Vec<(u64, Vec<u8>)> {
    TlogSlice::new(bytes)
        .map(|r| (r.timestamp_us, r.frame.as_bytes().to_vec()))
        .collect()
}

/// Serialize records back into a .tlog.
fn serialize(recs: &[(u64, Vec<u8>)]) -> Vec<u8> {
    let mut w = TlogWriter::new(Vec::new());
    for (ts, frame) in recs {
        w.write_frame(*ts, frame).unwrap();
    }
    w.into_inner()
}

fn py_key() -> [u8; 32] {
    core::array::from_fn(|i| i as u8)
}

#[test]
fn clean_log_is_intact_and_matches_the_stats() {
    let bytes = sample();
    let report = FlightReport::from_bytes(&bytes);

    assert_eq!(report.verdict(), Verdict::Intact);
    assert!(!report.integrity.is_tampered());
    assert!(report.integrity.tamper_window().is_none());
    assert_eq!(report.integrity.bad_crc, 0);
    assert_eq!(report.integrity.skipped_bytes, 0);
    assert!(report.integrity.anomalies.is_empty());

    // Frame count and senders agree with LinkStats over the same log.
    let mut stats = LinkStats::default();
    for r in TlogSlice::new(&bytes) {
        stats.observe(r.timestamp_us, &r.frame);
    }
    assert_eq!(report.records, 1426);
    assert_eq!(report.records, stats.total_received());
    let sources = stats.sources();
    assert_eq!(report.senders.len(), sources.len());
    for (sum, ((sys, comp), s)) in report.senders.iter().zip(sources) {
        assert_eq!((sum.sys_id, sum.comp_id), (sys, comp));
        assert_eq!(sum.received, s.received);
        assert_eq!(sum.lost, s.lost);
        // Messages add up to the frame count.
        let total: u64 = sum.messages.iter().map(|(_, n)| n).sum();
        assert_eq!(total, sum.received);
    }

    // The report renders without panicking and leads with the verdict.
    let text = format!("{report}");
    assert!(text.starts_with("INTEGRITY: intact"));
    assert!(text.contains("FLIGHT REPORT"));
    assert!(text.contains("sender 1/1"));
}

#[test]
fn reordered_frames_are_flagged_as_tampering() {
    // Take the real log and swap two adjacent records so a later timestamp
    // comes before an earlier one: exactly what splicing or reordering a
    // file does, and what a live logger never writes.
    let bytes = sample();
    let mut recs = records(&bytes);
    // Find an adjacent pair with strictly increasing timestamps to swap.
    let i = (0..recs.len() - 1)
        .find(|&i| recs[i].0 < recs[i + 1].0)
        .expect("a pair with increasing time");
    let (older, newer) = (recs[i].0, recs[i + 1].0);
    recs.swap(i, i + 1);
    let tampered = serialize(&recs);

    let report = FlightReport::from_bytes(&tampered);
    assert_eq!(report.verdict(), Verdict::Tampered);
    assert!(report.integrity.is_tampered());

    let regressions: Vec<_> = report
        .integrity
        .anomalies
        .iter()
        .filter(|a| a.kind == AnomalyKind::TimeRegression)
        .collect();
    assert_eq!(regressions.len(), 1, "exactly one backward time step");
    let a = regressions[0];
    assert_eq!(a.at_us, older); // the older frame now sits after the newer one
    assert_eq!(a.prev_us, newer);
    assert_eq!(a.magnitude, newer - older);

    let (from, to) = report.integrity.tamper_window().unwrap();
    assert_eq!((from, to), (older, newer));
    assert!(format!("{report}").contains("tampering"));
}

#[test]
fn edited_payload_breaks_the_checksum_and_reads_as_damaged() {
    // Flip one payload byte of one frame without fixing its checksum: the
    // frame no longer matches its message definition. This is what ordinary
    // corruption or a careless edit looks like, so it is reported as damage,
    // not as proof of tampering.
    let bytes = sample();
    let mut recs = records(&bytes);
    // Pick a frame with a non-empty payload and flip its first payload byte.
    let idx = recs
        .iter()
        .position(|(_, f)| {
            let fr = FrameRef::from_bytes_unchecked(f);
            fr.payload_len() > 0
        })
        .unwrap();
    let frame = &mut recs[idx].1;
    let payload_off = {
        let fr = FrameRef::from_bytes_unchecked(frame);
        match fr.version() {
            Version::V1 => 6,
            Version::V2 => 10,
        }
    };
    frame[payload_off] ^= 0xFF;
    let edited = serialize(&recs);

    let report = FlightReport::from_bytes(&edited);
    assert_eq!(report.verdict(), Verdict::Damaged);
    assert!(!report.integrity.is_tampered());
    assert!(report.integrity.bad_crc >= 1);
    assert!(report
        .integrity
        .anomalies
        .iter()
        .any(|a| a.kind == AnomalyKind::ChecksumFailure));
    assert!(format!("{report}").contains("failed their checksum"));
}

#[test]
fn a_fully_signed_log_verifies_as_intact_and_signed() {
    // Build a small log of valid MAVLink 2 signed frames (one key, rising
    // signing timestamps so none is a replay) and verify it with the key.
    let key = py_key();
    let payload = [0u8, 0, 0, 0, 2, 3, 0x51, 3, 3]; // a HEARTBEAT payload
    let mut w = TlogWriter::new(Vec::new());
    for n in 0..5u64 {
        let header = Header {
            seq: n as u8,
            sys_id: 1,
            comp_id: 1,
            msg_id: 0,
        };
        let mut buf = [0u8; 64];
        let len = encode_signed(header, &payload, None, &key, 0, 1_000 + n, &mut buf).unwrap();
        w.write_frame(1_700_000_000_000_000 + n, &buf[..len])
            .unwrap();
    }
    let signed = w.into_inner();

    let report = FlightReport::from_bytes_with_key(&signed, key);
    assert_eq!(report.verdict(), Verdict::IntactSigned);
    assert!(report.integrity.signing_checked);
    assert_eq!(report.integrity.signed_valid, 5);
    assert_eq!(report.integrity.bad_signatures, 0);
    assert_eq!(report.integrity.replays, 0);
    assert_eq!(report.integrity.unsigned_from_signers, 0);
    assert!(!report.integrity.is_tampered());

    // Without the key, the same log is only "intact": nothing was verified.
    assert_eq!(FlightReport::from_bytes(&signed).verdict(), Verdict::Intact);
}

#[test]
fn a_forged_signed_frame_is_flagged_as_tampering() {
    // A signed frame whose payload was changed and checksum fixed up, as an
    // attacker without the key would leave it: the frame passes its CRC but
    // its signature no longer verifies.
    let key = py_key();
    let payload = [0u8, 0, 0, 0, 2, 3, 0x51, 3, 3];
    let header = Header {
        seq: 0,
        sys_id: 1,
        comp_id: 1,
        msg_id: 0,
    };
    let mut buf = [0u8; 64];
    let len = encode_signed(header, &payload, None, &key, 0, 1_000, &mut buf).unwrap();
    let mut frame = buf[..len].to_vec();
    // Change a payload byte and repair the checksum so the frame still parses.
    frame[14] ^= 0x20;
    let fr = FrameRef::from_bytes_unchecked(&frame);
    let end = 10 + fr.payload_len() as usize;
    let crc = fr.compute_crc(message_info(0).unwrap().crc_extra);
    frame[end..end + 2].copy_from_slice(&crc.to_le_bytes());

    let mut w = TlogWriter::new(Vec::new());
    w.write_frame(1_700_000_000_000_000, &frame).unwrap();
    let forged = w.into_inner();

    // The frame parses cleanly (CRC is valid): no damage.
    let report = FlightReport::from_bytes_with_key(&forged, key);
    assert_eq!(report.integrity.bad_crc, 0);
    assert_eq!(report.integrity.skipped_bytes, 0);
    assert_eq!(report.verdict(), Verdict::Tampered);
    assert_eq!(report.integrity.bad_signatures, 1);
    assert!(report
        .integrity
        .anomalies
        .iter()
        .any(|a| a.kind == AnomalyKind::BadSignature));
    assert!(format!("{report}").contains("forged signature"));
}

// The CLI `report` command.

fn cli(args: &[&str]) -> (Option<i32>, String) {
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_mavlink-tlog"))
        .args(args)
        .output()
        .unwrap();
    (out.status.code(), String::from_utf8(out.stdout).unwrap())
}

const LOG: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/data/rust-mavlink-log.tlog"
);

#[test]
fn cli_report_on_clean_log_exits_zero() {
    let (code, out) = cli(&["report", LOG]);
    assert_eq!(code, Some(0));
    assert!(out.contains("INTEGRITY: intact"));
    assert!(out.contains("FLIGHT REPORT"));
    assert!(out.contains("sender 1/1"));
}

#[test]
fn cli_report_on_reordered_log_exits_three() {
    let bytes = sample();
    let mut recs = records(&bytes);
    let i = (0..recs.len() - 1)
        .find(|&i| recs[i].0 < recs[i + 1].0)
        .unwrap();
    recs.swap(i, i + 1);
    let tampered = serialize(&recs);
    let path =
        std::env::temp_dir().join(format!("mavlink-tlog-report-{}.tlog", std::process::id()));
    std::fs::write(&path, &tampered).unwrap();
    let (code, out) = cli(&["report", path.to_str().unwrap()]);
    std::fs::remove_file(&path).unwrap();
    assert_eq!(code, Some(3), "tampered log should exit 3");
    assert!(out.contains("tampering"));
}
