//! Damage, chunked reads, encoding vectors and the writer.

use mavlink_tlog::*;
use std::io::Read;

fn sample() -> Vec<u8> {
    std::fs::read(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/data/rust-mavlink-log.tlog"
    ))
    .unwrap()
}

fn all(bytes: &[u8]) -> (Vec<(u64, Vec<u8>)>, ReadStats) {
    let mut s = TlogSlice::new(bytes);
    let v = s
        .by_ref()
        .map(|r| (r.timestamp_us, r.frame.as_bytes().to_vec()))
        .collect();
    (v, s.stats())
}

/// Byte offsets where each record starts.
fn offsets(bytes: &[u8]) -> Vec<usize> {
    let mut s = TlogSlice::new(bytes);
    let mut out = vec![0];
    while s.next().is_some() {
        out.push(s.position());
    }
    out.pop();
    out
}

#[test]
fn pymavlink_vectors() {
    // Encoded by pymavlink 2.4.50 (see tools/pymavlink_reference.py).
    let hb_v1 = hex("fe09070101000400000002035103036391");
    let hb_v2 = hex("fd0900000701010000000400000002035103033d90");
    let gps_v2 =
        hex("fd1e000008010118000015cd5b07000000004a52401c43f4170520a1070078009600dc052823030cec86");
    for (bytes, version, id, seq) in [
        (&hb_v1, Version::V1, 0, 7),
        (&hb_v2, Version::V2, 0, 7),
        (&gps_v2, Version::V2, 24, 8),
    ] {
        let f = FrameRef::parse(bytes, CrcPolicy::Strict).unwrap();
        assert_eq!(
            (f.version(), f.msg_id(), f.seq(), f.sys_id(), f.comp_id()),
            (version, id, seq, 1, 1)
        );
        assert_eq!(f.len(), bytes.len());
        // Re-encoding the payload gives the same bytes.
        let mut out = [0u8; 300];
        let header = Header {
            seq: f.seq(),
            sys_id: 1,
            comp_id: 1,
            msg_id: id,
        };
        let n = encode(version, header, f.payload(), None, &mut out).unwrap();
        assert_eq!(&out[..n], &bytes[..]);
    }
    assert_eq!(
        FrameRef::parse(&hb_v2, CrcPolicy::Strict)
            .unwrap()
            .message_name(),
        Some("HEARTBEAT")
    );
    let mut bad = hb_v2.clone();
    bad[12] ^= 1;
    assert!(matches!(
        FrameRef::parse(&bad, CrcPolicy::Strict),
        Err(FrameError::BadCrc { .. })
    ));
    assert!(FrameRef::parse(&bad, CrcPolicy::Off).is_ok());
}

#[test]
fn v2_trailing_zero_truncation() {
    let mut payload = [0u8; 52];
    payload[..4].copy_from_slice(&[1, 2, 3, 4]);
    let mut out = [0u8; 300];
    let n = encode(
        Version::V2,
        Header {
            msg_id: 24,
            ..Header::default()
        },
        &payload,
        None,
        &mut out,
    )
    .unwrap();
    let f = FrameRef::parse(&out[..n], CrcPolicy::Strict).unwrap();
    assert_eq!(f.payload(), &[1, 2, 3, 4]);
    let mut full = [0u8; 255];
    assert_eq!(f.payload_padded(&mut full), 52);
    assert_eq!(&full[..52], &payload);
}

#[test]
fn flipped_byte_loses_only_that_record() {
    let clean = sample();
    let (good, _) = all(&clean);
    let offs = offsets(&clean);
    for &k in &[1usize, 50, 400, 1000, 1400] {
        let mut bad = clean.clone();
        bad[offs[k] + 8 + 6] ^= 0x5A; // a payload or header byte of record k
        let (got, stats) = all(&bad);
        let mut expect = good.clone();
        expect.remove(k);
        assert_eq!(got, expect, "record {k}");
        assert!(stats.skipped_bytes > 0);
    }
}

#[test]
fn garbage_between_records_is_skipped() {
    let clean = sample();
    let (good, _) = all(&clean);
    let offs = offsets(&clean);
    let mut bad = clean[..offs[700]].to_vec();
    bad.extend_from_slice(&[0xFD, 0x09, 0x00, 0xFE, 0x55, 0xAA, 0xFD, 0xFD, 0x01]);
    bad.extend_from_slice(&clean[offs[700]..]);
    let (got, stats) = all(&bad);
    assert_eq!(got, good);
    assert_eq!(stats.skipped_bytes, 9);
}

#[test]
fn truncated_file() {
    let clean = sample();
    let (good, _) = all(&clean);
    let cut = &clean[..clean.len() - 5];
    let (got, stats) = all(cut);
    assert_eq!(got.len(), good.len() - 1);
    assert!(stats.truncated_tail + stats.skipped_bytes > 0);
}

/// Hands out at most `n` bytes per read, to exercise refills mid-record.
struct Trickle<'a>(&'a [u8], usize, usize);
impl Read for Trickle<'_> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.2 = self.2 % 7 + 1;
        let n = buf.len().min(self.0.len()).min(self.1.min(self.2));
        buf[..n].copy_from_slice(&self.0[..n]);
        self.0 = &self.0[n..];
        Ok(n)
    }
}

#[test]
fn streaming_reader_matches_slice_reader() {
    let mut damaged = sample();
    for i in (100..damaged.len()).step_by(5_003) {
        damaged[i] ^= 0xFF;
    }
    let (want, want_stats) = all(&damaged);
    for chunk in [1, 3, 64, 100_000] {
        let mut r = TlogReader::new(Trickle(&damaged, chunk, 0));
        let got: Vec<_> = r
            .by_ref()
            .map(|x| x.map(|o| (o.timestamp_us, o.frame)).unwrap())
            .collect();
        assert_eq!(got, want, "chunk {chunk}");
        assert_eq!(r.stats(), want_stats, "chunk {chunk}");
    }
}

#[test]
fn writer_roundtrip() {
    let clean = sample();
    let mut w = TlogWriter::new(Vec::new());
    for r in TlogSlice::new(&clean) {
        w.write_frame(r.timestamp_us, r.frame.as_bytes()).unwrap();
    }
    assert_eq!(w.records(), 1426);
    assert_eq!(
        w.into_inner(),
        clean,
        "rewriting a clean log reproduces it byte for byte"
    );
    let mut w = TlogWriter::new(Vec::new());
    assert!(w.write_frame(0, &[0xFE, 9, 0]).is_err());
}

#[test]
fn random_input_never_panics() {
    let mut x: u64 = 0x1234_5678_9abc_def0;
    let mut buf = vec![0u8; 4096];
    for _ in 0..2000 {
        for b in buf.iter_mut() {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            *b = match x % 5 {
                0 => 0xFD,
                1 => 0xFE,
                _ => x as u8,
            };
        }
        for policy in [CrcPolicy::Strict, CrcPolicy::Lenient, CrcPolicy::Off] {
            let mut s = TlogSlice::with_options(
                &buf,
                ReadOptions {
                    crc: policy,
                    ..ReadOptions::default()
                },
            );
            let mut stats = LinkStats::default();
            for r in s.by_ref() {
                stats.observe(r.timestamp_us, &r.frame);
                let _ = (
                    r.frame.payload(),
                    r.frame.signature(),
                    r.frame.message_name(),
                );
            }
            let st = s.stats();
            assert!(st.skipped_bytes + st.truncated_tail <= buf.len() as u64);
        }
    }
}

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}
