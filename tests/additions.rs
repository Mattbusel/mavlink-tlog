//! MAVLink 2 signing, the follow-mode reader, dropout stats and the CLI.

use mavlink_tlog::*;
use std::cell::RefCell;
use std::io::{self, Read};
use std::rc::Rc;

fn sample() -> Vec<u8> {
    std::fs::read(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/data/rust-mavlink-log.tlog"
    ))
    .unwrap()
}

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

// Signed by pymavlink 2.4.50 (MAVLINK20=1, common dialect) with
// secret_key = bytes(range(32)), link_id = 3, timestamp 0x0123456789AB,
// srcSystem = srcComponent = 1, seq 0 (pack() does not advance seq).
// pymavlink bumps the timestamp by one per
// signed frame.
const PY_HEARTBEAT: &str = "fd0901000001010000000300000002035103036c1e03ab896745230195fccf3ad831";
const PY_COMMAND_LONG: &str = "fd2001000001014c00000000803f00000000000000000000000000000000000000000000000090010101ec6d03ac89674523010b6d7ba8491e";

fn py_key() -> [u8; 32] {
    core::array::from_fn(|i| i as u8)
}

#[test]
fn verifies_pymavlink_signatures() {
    let key = py_key();
    for (h, ts) in [
        (PY_HEARTBEAT, 0x0123456789AB),
        (PY_COMMAND_LONG, 0x0123456789AC),
    ] {
        let bytes = hex(h);
        let f = FrameRef::parse(&bytes, CrcPolicy::Strict).unwrap();
        assert_eq!(f.len(), bytes.len());
        let sig = f.signature_fields().unwrap();
        assert_eq!(sig.link_id, 3);
        assert_eq!(sig.timestamp, ts);
        assert!(f.verify_signature(&key));
        let mut wrong = key;
        wrong[31] ^= 1;
        assert!(!f.verify_signature(&wrong));
    }
}

#[test]
fn encode_signed_matches_pymavlink_byte_for_byte() {
    let key = py_key();
    for (h, seq, ts) in [
        (PY_HEARTBEAT, 0, 0x0123456789AB),
        (PY_COMMAND_LONG, 0, 0x0123456789AC),
    ] {
        let want = hex(h);
        let f = FrameRef::from_bytes_unchecked(&want);
        let header = Header {
            seq,
            sys_id: 1,
            comp_id: 1,
            msg_id: f.msg_id(),
        };
        let mut out = [0u8; 300];
        let n = encode_signed(header, f.payload(), None, &key, 3, ts, &mut out).unwrap();
        assert_eq!(&out[..n], &want[..]);
    }
}

#[test]
fn tampered_signed_frame_fails_signature() {
    let key = py_key();
    let mut bytes = hex(PY_COMMAND_LONG);
    // Change param1 and fix the CRC, as an attacker without the key would.
    bytes[13] ^= 0x40;
    let f = FrameRef::from_bytes_unchecked(&bytes);
    let end = 10 + f.payload_len() as usize;
    let crc = f.compute_crc(message_info(76).unwrap().crc_extra);
    bytes[end..end + 2].copy_from_slice(&crc.to_le_bytes());
    let f = FrameRef::parse(&bytes, CrcPolicy::Strict).unwrap();
    assert!(!f.verify_signature(&key));
    let mut v = SigningVerifier::new(key);
    assert_eq!(v.check(&f), SignatureStatus::BadSignature);
}

#[test]
fn verifier_rejects_replays_per_stream() {
    let key = py_key();
    let hb = hex(PY_HEARTBEAT);
    let cmd = hex(PY_COMMAND_LONG);
    let mut v = SigningVerifier::new(key);
    let hb = FrameRef::from_bytes_unchecked(&hb);
    let cmd = FrameRef::from_bytes_unchecked(&cmd);
    assert_eq!(v.check(&hb), SignatureStatus::Valid);
    assert_eq!(v.check(&cmd), SignatureStatus::Valid);
    // The heartbeat is older than the last accepted frame on 1/1 link 3.
    assert_eq!(v.check(&hb), SignatureStatus::Replay);
    assert_eq!(v.check(&cmd), SignatureStatus::Replay);
    assert_eq!(v.last_timestamp(1, 1, 3), Some(0x0123456789AC));

    // Another link id is its own stream.
    let mut buf = [0u8; 300];
    let header = Header {
        seq: 2,
        sys_id: 1,
        comp_id: 1,
        msg_id: 0,
    };
    let n = encode_signed(header, hb.payload(), None, &key, 4, 5, &mut buf).unwrap();
    assert_eq!(
        v.check(&FrameRef::from_bytes_unchecked(&buf[..n])),
        SignatureStatus::Valid
    );

    let unsigned = hex("fd0900000701010000000400000002035103033d90");
    assert_eq!(
        v.check(&FrameRef::from_bytes_unchecked(&unsigned)),
        SignatureStatus::Unsigned
    );
}

#[test]
fn signing_timestamp_to_unix() {
    let s = FrameRef::from_bytes_unchecked(&hex(PY_HEARTBEAT))
        .signature_fields()
        .unwrap();
    assert_eq!(s.unix_us(), SIGNING_EPOCH_UNIX_US + 0x0123456789AB * 10);
    let mut out = [0u8; 300];
    let h = Header::default();
    assert!(encode_signed(h, &[0; 9], None, &[0; 32], 0, 1 << 48, &mut out).is_none());
    assert!(encode_signed(h, &[0; 9], None, &[0; 32], 0, 1, &mut out[..20]).is_none());
}

#[test]
fn signed_frames_read_from_a_tlog() {
    let mut w = TlogWriter::new(Vec::new());
    w.write_frame(1, &hex(PY_HEARTBEAT)).unwrap();
    w.write_frame(2, &hex(PY_COMMAND_LONG)).unwrap();
    let bytes = w.into_inner();
    let mut log = TlogSlice::with_options(
        &bytes,
        ReadOptions {
            crc: CrcPolicy::Strict,
            ..ReadOptions::default()
        },
    );
    let mut v = SigningVerifier::new(py_key());
    let got: Vec<_> = log.by_ref().map(|r| v.check(&r.frame)).collect();
    assert_eq!(got, [SignatureStatus::Valid, SignatureStatus::Valid]);
    assert_eq!(log.stats().skipped_bytes, 0);
}

/// A `Read` over a buffer another party keeps appending to, like a tlog
/// file a ground station is still writing.
#[derive(Clone, Default)]
struct Growing(Rc<RefCell<(Vec<u8>, usize)>>);

impl Growing {
    fn push(&self, b: &[u8]) {
        self.0.borrow_mut().0.extend_from_slice(b);
    }
}

impl Read for Growing {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        let mut g = self.0.borrow_mut();
        let (buf, pos) = &mut *g;
        let n = (buf.len() - *pos).min(out.len());
        out[..n].copy_from_slice(&buf[*pos..*pos + n]);
        *pos += n;
        Ok(n)
    }
}

#[test]
fn follow_reads_a_growing_log_without_losing_partial_records() {
    let bytes = sample();
    let want: Vec<_> = TlogSlice::new(&bytes)
        .map(|r| (r.timestamp_us, r.frame.as_bytes().to_vec()))
        .collect();
    for step in [1usize, 7, 33, 500, 4096] {
        let src = Growing::default();
        let mut log = TlogReader::follow(src.clone());
        assert!(log.is_following());
        let mut got = Vec::new();
        for chunk in bytes.chunks(step) {
            src.push(chunk);
            while let Some(r) = log.next_record().unwrap() {
                got.push((r.timestamp_us, r.frame.as_bytes().to_vec()));
            }
            assert_eq!(log.stats().skipped_bytes, 0, "step {step}");
            assert_eq!(log.stats().truncated_tail, 0, "step {step}");
        }
        assert_eq!(got, want, "step {step}");
        assert_eq!(log.pending_bytes(), 0);
    }
}

#[test]
fn follow_then_finish_counts_the_unfinished_record() {
    let bytes = sample();
    let cut = bytes.len() - 5;
    let src = Growing::default();
    src.push(&bytes[..cut]);
    let mut log = TlogReader::follow(src.clone());
    let mut n = 0;
    while log.next_record().unwrap().is_some() {
        n += 1;
    }
    let pending = log.pending_bytes();
    assert!(pending > 0);
    assert_eq!(log.stats().truncated_tail, 0);
    // Still nothing new: still waiting, nothing lost.
    assert!(log.next_record().unwrap().is_none());
    assert_eq!(log.pending_bytes(), pending);

    log.finish();
    assert!(!log.is_following());
    assert!(log.next_record().unwrap().is_none());
    let s = log.stats();
    assert_eq!(s.records, n);
    assert_eq!(s.skipped_bytes + s.truncated_tail, pending as u64);
    assert_eq!(n as usize + 1, TlogSlice::new(&bytes).count());
}

#[test]
fn longest_gap_finds_the_dropout() {
    let hb = hex("fe09070101000400000002035103036391");
    let mut stats = LinkStats::default();
    let f = FrameRef::from_bytes_unchecked(&hb);
    for ts in [
        1_000_000, 1_100_000, 1_200_000, 4_200_000, 4_300_000, 4_250_000,
    ] {
        stats.observe(ts, &f);
    }
    let s = stats.source(1, 1).unwrap();
    assert_eq!(s.longest_gap_us, 3_000_000);
    assert_eq!(s.longest_gap_end_us, 4_200_000);
    let r = s.rate_total_hz().unwrap();
    assert!((r - 5.0 / 3.3).abs() < 1e-9);
}

#[test]
fn longest_gap_on_real_log_is_bounded_by_span() {
    let bytes = sample();
    let mut stats = LinkStats::default();
    for r in TlogSlice::new(&bytes) {
        stats.observe(r.timestamp_us, &r.frame);
    }
    for (_, s) in stats.sources() {
        if s.received > 1 {
            assert!(s.longest_gap_us > 0);
            assert!(s.longest_gap_us <= s.last_us - s.first_us);
            assert!(s.longest_gap_end_us > s.first_us && s.longest_gap_end_us <= s.last_us);
        }
    }
}

// The CLI binary, run on the test log.

fn cli(args: &[&str]) -> (bool, String, String) {
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_mavlink-tlog"))
        .args(args)
        .output()
        .unwrap();
    (
        out.status.success(),
        String::from_utf8(out.stdout).unwrap(),
        String::from_utf8(out.stderr).unwrap(),
    )
}

const LOG: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/data/rust-mavlink-log.tlog"
);

#[test]
fn cli_csv_has_one_row_per_frame() {
    let n = TlogSlice::new(&sample()).count();
    let (ok, out, _) = cli(&["csv", LOG]);
    assert!(ok);
    let mut lines = out.lines();
    assert!(lines
        .next()
        .unwrap()
        .starts_with("timestamp_us,time_s,sys_id"));
    let rows: Vec<_> = lines.collect();
    assert_eq!(rows.len(), n);
    assert!(rows[0].contains(",0.000000,"));
    assert!(rows.iter().all(|r| r.split(',').count() == 11));
}

#[test]
fn cli_filters_by_message_and_sender() {
    let bytes = sample();
    let want = TlogSlice::new(&bytes)
        .filter(|r| r.frame.msg_id() == 0 && r.frame.sys_id() == 1)
        .count();
    assert!(want > 0);
    let (ok, out, _) = cli(&["json", LOG, "--msg", "heartbeat", "--sys", "1"]);
    assert!(ok);
    assert_eq!(out.lines().count(), want);
    for l in out.lines() {
        let v: serde_json::Value = serde_json::from_str(l).unwrap();
        assert_eq!(v["msg_name"], "HEARTBEAT");
        assert_eq!(v["sys_id"], 1);
        assert_eq!(v["signed"], false);
    }
}

#[test]
fn cli_slice_cuts_a_time_window() {
    let bytes = sample();
    let first = TlogSlice::new(&bytes).next().unwrap().timestamp_us;
    let want: Vec<_> = TlogSlice::new(&bytes)
        .filter(|r| {
            let rel = (r.timestamp_us - first) as f64 / 1e6;
            (1.0..=2.5).contains(&rel)
        })
        .map(|r| (r.timestamp_us, r.frame.as_bytes().to_vec()))
        .collect();
    assert!(!want.is_empty());
    let out = std::env::temp_dir().join(format!("mavlink-tlog-slice-{}.tlog", std::process::id()));
    let out_s = out.to_str().unwrap();
    let (ok, _, err) = cli(&["slice", LOG, out_s, "--from", "1", "--to", "2.5"]);
    assert!(ok, "{err}");
    let cut = std::fs::read(&out).unwrap();
    std::fs::remove_file(&out).unwrap();
    let got: Vec<_> = TlogSlice::new(&cut)
        .map(|r| (r.timestamp_us, r.frame.as_bytes().to_vec()))
        .collect();
    assert_eq!(got, want);
}

#[test]
fn cli_signing_flags_unsigned_and_replayed_frames() {
    let mut w = TlogWriter::new(Vec::new());
    w.write_frame(1, &hex(PY_HEARTBEAT)).unwrap();
    w.write_frame(2, &hex(PY_COMMAND_LONG)).unwrap();
    w.write_frame(3, &hex(PY_COMMAND_LONG)).unwrap(); // replay
    w.write_frame(4, &hex("fd0900000701010000000400000002035103033d90"))
        .unwrap(); // unsigned heartbeat from the same sender
    let path = std::env::temp_dir().join(format!("mavlink-tlog-sign-{}.tlog", std::process::id()));
    std::fs::write(&path, w.into_inner()).unwrap();
    let key: String = (0..32).map(|i| format!("{i:02x}")).collect();
    let (ok, out, err) = cli(&["signing", path.to_str().unwrap(), "--key", &key]);
    std::fs::remove_file(&path).unwrap();
    assert!(ok, "{err}");
    assert!(out.contains("sender 1/1: 2 valid, 0 bad signature, 1 replayed, 1 unsigned"));
    assert!(out.contains("HEARTBEAT"));
    assert!(out.contains("FOUND"));
}

#[test]
fn cli_stats_and_errors() {
    let (ok, out, _) = cli(&["stats", LOG]);
    assert!(ok);
    assert!(out.contains("sender 1/1"));
    assert!(out.contains("longest silence"));
    let (ok, _, err) = cli(&["csv", LOG, "--msg", "NOT_A_MESSAGE"]);
    assert!(!ok);
    assert!(err.contains("unknown message"));
    let (ok, _, _) = cli(&["signing", LOG, "--key", "abc"]);
    assert!(!ok);
}
