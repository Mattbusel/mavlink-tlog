//! The .tlog container: an 8 byte big-endian timestamp (microseconds since
//! the Unix epoch) before every MAVLink frame, as written by QGroundControl,
//! Mission Planner and MAVProxy.

use crate::frame::{CrcPolicy, FrameError, FrameRef};

/// Length of the timestamp before every frame.
pub const TIMESTAMP_LEN: usize = 8;

/// One record: a frame and the time the logger received it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Record<'a> {
    /// Microseconds since the Unix epoch, as written by the logger.
    pub timestamp_us: u64,
    /// The MAVLink frame.
    pub frame: FrameRef<'a>,
}

/// Options for reading.
#[derive(Debug, Clone, Copy)]
pub struct ReadOptions {
    /// Checksum policy for frames.
    pub crc: CrcPolicy,
    /// When a frame's checksum cannot be verified (unknown id under
    /// [`CrcPolicy::Lenient`], or [`CrcPolicy::Off`]), its timestamp must be
    /// within this many microseconds of the previous record, or it is taken
    /// for a false start while resynchronizing. Default: one day.
    pub unverified_window_us: u64,
}

impl Default for ReadOptions {
    fn default() -> Self {
        ReadOptions {
            crc: CrcPolicy::Lenient,
            unverified_window_us: 86_400_000_000,
        }
    }
}

/// What the reader had to skip.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ReadStats {
    /// Records returned.
    pub records: u64,
    /// Bytes skipped while looking for the next valid record.
    pub skipped_bytes: u64,
    /// Places where a frame structure was found but its checksum was wrong.
    pub bad_crc: u64,
    /// Places where a frame named a message id not in the table (strict policy).
    pub unknown_messages: u64,
    /// Bytes at the end of the input that did not form a complete record.
    pub truncated_tail: u64,
}

pub(crate) enum Attempt<'a> {
    Ok(Record<'a>, usize),
    Incomplete,
    Invalid(Option<FrameError>),
}

pub(crate) fn try_record<'a>(
    buf: &'a [u8],
    opts: &ReadOptions,
    prev_ts: Option<u64>,
) -> Attempt<'a> {
    if buf.len() < TIMESTAMP_LEN + 1 {
        return Attempt::Incomplete;
    }
    let ts = u64::from_be_bytes(buf[..TIMESTAMP_LEN].try_into().expect("8 bytes"));
    let frame = match FrameRef::parse_unchecked(&buf[TIMESTAMP_LEN..]) {
        Ok(f) => f,
        Err(FrameError::Incomplete { .. }) => return Attempt::Incomplete,
        Err(e) => return Attempt::Invalid(Some(e)),
    };
    let verified = match opts.crc {
        CrcPolicy::Off => false,
        CrcPolicy::Lenient => crate::frame::message_info(frame.msg_id()).is_some(),
        CrcPolicy::Strict | CrcPolicy::Custom(_) => true,
    };
    if let Err(e) = frame.check(opts.crc) {
        return Attempt::Invalid(Some(e));
    }
    if !verified {
        if let Some(p) = prev_ts {
            if ts.abs_diff(p) > opts.unverified_window_us {
                return Attempt::Invalid(None);
            }
        }
    }
    Attempt::Ok(
        Record {
            timestamp_us: ts,
            frame,
        },
        TIMESTAMP_LEN + frame.len(),
    )
}

pub(crate) fn note_invalid(stats: &mut ReadStats, e: Option<FrameError>) {
    match e {
        Some(FrameError::BadCrc { .. }) | Some(FrameError::PayloadTooLong { .. }) => {
            stats.bad_crc += 1
        }
        Some(FrameError::UnknownMessage(_)) => stats.unknown_messages += 1,
        _ => {}
    }
}

/// Reads records from a tlog held in memory, without allocating.
///
/// On a damaged record it skips forward one byte at a time until a valid
/// record starts, and counts what it skipped in [`TlogSlice::stats`].
///
/// ```
/// use mavlink_tlog::TlogSlice;
///
/// let bytes = std::fs::read("tests/data/rust-mavlink-log.tlog").unwrap();
/// let mut log = TlogSlice::new(&bytes);
/// let heartbeats = log.by_ref().filter(|r| r.frame.msg_id() == 0).count();
/// assert!(heartbeats > 0);
/// assert_eq!(log.stats().skipped_bytes, 0);
/// ```
#[derive(Debug, Clone)]
pub struct TlogSlice<'a> {
    buf: &'a [u8],
    pos: usize,
    opts: ReadOptions,
    prev_ts: Option<u64>,
    stats: ReadStats,
}

impl<'a> TlogSlice<'a> {
    /// A reader with default options.
    pub fn new(buf: &'a [u8]) -> Self {
        Self::with_options(buf, ReadOptions::default())
    }

    /// A reader with the given options.
    pub fn with_options(buf: &'a [u8], opts: ReadOptions) -> Self {
        TlogSlice {
            buf,
            pos: 0,
            opts,
            prev_ts: None,
            stats: ReadStats::default(),
        }
    }

    /// What has been skipped so far.
    pub fn stats(&self) -> ReadStats {
        self.stats
    }

    /// Byte offset of the next unread record.
    pub fn position(&self) -> usize {
        self.pos
    }
}

impl<'a> Iterator for TlogSlice<'a> {
    type Item = Record<'a>;

    fn next(&mut self) -> Option<Record<'a>> {
        loop {
            match try_record(&self.buf[self.pos..], &self.opts, self.prev_ts) {
                Attempt::Ok(rec, used) => {
                    self.pos += used;
                    self.prev_ts = Some(rec.timestamp_us);
                    self.stats.records += 1;
                    return Some(rec);
                }
                Attempt::Incomplete => {
                    let rest = (self.buf.len() - self.pos) as u64;
                    // A damaged length byte can make a record look longer
                    // than the input: keep scanning past it.
                    if rest > (TIMESTAMP_LEN + 1) as u64 {
                        self.pos += 1;
                        self.stats.skipped_bytes += 1;
                        continue;
                    }
                    self.stats.truncated_tail += rest;
                    self.pos = self.buf.len();
                    return None;
                }
                Attempt::Invalid(e) => {
                    note_invalid(&mut self.stats, e);
                    self.pos += 1;
                    self.stats.skipped_bytes += 1;
                }
            }
        }
    }
}
