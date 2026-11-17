//! Every number this crate reports for a real log must match pymavlink.
//!
//! `tests/data/*.pymavlink.json` were made by `tools/pymavlink_reference.py`
//! (pymavlink 2.4.50, `all` dialect, robust parsing): message counts per id
//! and per sender, first and last timestamps, and the sequence statistics
//! computed from pymavlink's decoded messages.
//!
//! Logs:
//! * `rust-mavlink-log.tlog`: from mavlink/rust-mavlink (MIT OR Apache-2.0), committed.
//! * `pymavlink-flight.tlog`: a 40 minute ArduPilot flight from
//!   ArduPilot/pymavlink (5.9 MB). Downloaded by CI into tests/data/external.
//! * `mprust-autotest.tlog`: an ArduPilot SITL autotest log with damaged
//!   records, from davidbuzz/MissionPlannerRust. Downloaded by CI.
//!
//! The downloaded logs are skipped (with a message) when absent.

use mavlink_tlog::{CrcPolicy, LinkStats, ReadOptions, TlogReader, TlogSlice};
use serde_json::Value;
use std::collections::BTreeMap;
use std::path::PathBuf;

fn data(p: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(p)
}

/// Where this crate and pymavlink are known to disagree on a log, and why.
#[derive(Default)]
struct Differences {
    /// Messages pymavlink invents from a damaged header (id -2, "UNKNOWN_*").
    pymavlink_bogus: u64,
    /// Real frames, per message id, that pymavlink drops after damage.
    pymavlink_dropped: &'static [(u32, u64)],
}

fn check(log: &str, path: PathBuf) {
    check_with(log, path, Differences::default())
}

fn check_with(log: &str, path: PathBuf, diff: Differences) {
    if !path.exists() {
        eprintln!(
            "SKIP {log}: {} not present (run tools/fetch_logs.sh)",
            path.display()
        );
        return;
    }
    let reference: Value = serde_json::from_str(
        &std::fs::read_to_string(data(&log.replace(".tlog", ".pymavlink.json"))).unwrap(),
    )
    .unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let opts = ReadOptions {
        crc: CrcPolicy::Strict,
        ..ReadOptions::default()
    };

    let mut slice = TlogSlice::with_options(&bytes, opts);
    let mut stats = LinkStats::default();
    let mut by_id: BTreeMap<String, u64> = BTreeMap::new();
    let (mut first, mut last) = (None, 0);
    for rec in slice.by_ref() {
        stats.observe(rec.timestamp_us, &rec.frame);
        *by_id.entry(rec.frame.msg_id().to_string()).or_default() += 1;
        first.get_or_insert(rec.timestamp_us);
        last = rec.timestamp_us;
    }
    let read = slice.stats();
    println!("{log}: {read:?}");

    let dropped: u64 = diff.pymavlink_dropped.iter().map(|(_, n)| n).sum();
    assert_eq!(
        stats.total_received(),
        reference["messages"].as_u64().unwrap() - diff.pymavlink_bogus + dropped,
        "{log}: message count"
    );
    let mut ref_ids: BTreeMap<String, u64> = reference["by_msgid"]
        .as_object()
        .unwrap()
        .iter()
        .filter(|(k, _)| k.as_str() != "-2")
        .map(|(k, v)| (k.clone(), v.as_u64().unwrap()))
        .collect();
    for (id, n) in diff.pymavlink_dropped {
        *ref_ids.get_mut(&id.to_string()).unwrap() += n;
    }
    assert_eq!(by_id, ref_ids, "{log}: counts per message id");
    if diff.pymavlink_bogus == 0 {
        assert_eq!(
            first,
            reference["first_us"].as_u64(),
            "{log}: first timestamp"
        );
    } else {
        // pymavlink drops the very first frame of this log (see below).
        assert!(first.unwrap() < reference["first_us"].as_u64().unwrap());
    }
    assert_eq!(
        last,
        reference["last_us"].as_u64().unwrap(),
        "{log}: last timestamp"
    );

    if diff.pymavlink_bogus > 0 || dropped > 0 {
        // Sequence statistics shift once frames differ; the reader
        // agreement below still has to hold.
        let ours = stats.source(1, 1).unwrap();
        assert!(
            ours.lost <= reference["lost"]["1/1"].as_u64().unwrap(),
            "recovered frames can only lower the loss"
        );
        compare_readers(&path, opts, &stats, read);
        return;
    }
    for ((sys, comp), s) in stats.sources() {
        let key = format!("{sys}/{comp}");
        let get = |field: &str| {
            reference[field]
                .get(&key)
                .and_then(Value::as_u64)
                .unwrap_or(0)
        };
        assert_eq!(s.received, get("by_source"), "{log} {key}: received");
        assert_eq!(s.lost, get("lost"), "{log} {key}: lost");
        assert_eq!(s.duplicates, get("duplicates"), "{log} {key}: duplicates");
        assert_eq!(
            s.out_of_order,
            get("out_of_order"),
            "{log} {key}: out of order"
        );
    }
    assert_eq!(
        stats.sources().len(),
        reference["by_source"].as_object().unwrap().len()
    );
    compare_readers(&path, opts, &stats, read);
}

/// The streaming reader must agree with the slice reader exactly.
fn compare_readers(
    path: &PathBuf,
    opts: ReadOptions,
    stats: &LinkStats,
    read: mavlink_tlog::ReadStats,
) {
    let mut reader = TlogReader::with_options(std::fs::File::open(path).unwrap(), opts);
    let mut n = 0u64;
    while let Some(rec) = reader.next_record().unwrap() {
        let _ = rec.frame.msg_id();
        n += 1;
    }
    assert_eq!(n, stats.total_received());
    assert_eq!(reader.stats(), read);
}

#[test]
fn rust_mavlink_log_matches_pymavlink() {
    check("rust-mavlink-log.tlog", data("rust-mavlink-log.tlog"));
    // A clean log: nothing skipped.
    let bytes = std::fs::read(data("rust-mavlink-log.tlog")).unwrap();
    let mut s = TlogSlice::new(&bytes);
    s.by_ref().for_each(drop);
    assert_eq!(s.stats().skipped_bytes, 0);
    assert_eq!(s.stats().truncated_tail, 0);
}

#[test]
fn ardupilot_flight_log_matches_pymavlink() {
    check(
        "pymavlink-flight.tlog",
        data("external/pymavlink-flight.tlog"),
    );
}

#[test]
fn damaged_sitl_log_matches_pymavlink() {
    // This log mixes SITL console text with MAVLink and has 5 places where a
    // frame header was cut off and a whole frame follows (`fd 09 00` then
    // `fd 09 00 00 ...`). pymavlink parses each of those as a bogus message
    // from system 0 with an id above 65000 and loses the frames that follow
    // the damage; this crate skips the 3 broken bytes and reads them.
    check_with(
        "mprust-autotest.tlog",
        data("external/mprust-autotest.tlog"),
        Differences {
            pymavlink_bogus: 5,
            pymavlink_dropped: &[(0, 11), (22, 10), (253, 1)],
        },
    );
}
