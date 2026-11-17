//! Per-sender link statistics from sequence numbers (needs the `std` feature).

use std::collections::{BTreeMap, HashMap};

use crate::frame::FrameRef;

/// Statistics for one sender (system id, component id).
///
/// Every MAVLink sender increments its 8 bit sequence number by one per
/// frame it sends on a link. From that:
///
/// * a step of 0 is a **duplicate** (the same frame arrived twice, for
///   example through two routers),
/// * a step of `n` from 1 to 128 means `n - 1` frames were **lost**,
/// * a step of more than 128 (a step backwards) is counted as **out of
///   order** and does not move the expected sequence.
///
/// A sender that talks on several links with one shared counter (common
/// for ground stations) shows loss on each link that does not see all of
/// its frames; that is real loss from the point of view of this link.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SourceStats {
    /// Frames received.
    pub received: u64,
    /// Frames estimated lost from sequence gaps.
    pub lost: u64,
    /// Frames with a repeated sequence number.
    pub duplicates: u64,
    /// Frames whose sequence number went backwards.
    pub out_of_order: u64,
    /// Timestamp of the first frame, microseconds.
    pub first_us: u64,
    /// Timestamp of the latest frame, microseconds.
    pub last_us: u64,
    /// Frames per message id.
    pub by_msg_id: BTreeMap<u32, u64>,
    last_seq: Option<u8>,
}

impl SourceStats {
    /// `lost / (received + lost)`, or 0 before any frame.
    pub fn loss_ratio(&self) -> f64 {
        let total = self.received + self.lost;
        if total == 0 {
            0.0
        } else {
            self.lost as f64 / total as f64
        }
    }

    /// Average rate of one message id in Hz over this sender's time span.
    pub fn rate_hz(&self, msg_id: u32) -> Option<f64> {
        let n = *self.by_msg_id.get(&msg_id)?;
        let span = self.last_us.checked_sub(self.first_us)?;
        if span == 0 || n < 2 {
            return None;
        }
        Some((n - 1) as f64 / (span as f64 / 1e6))
    }
}

/// Link statistics for every sender seen.
///
/// ```
/// use mavlink_tlog::{LinkStats, TlogSlice};
///
/// let bytes = std::fs::read("tests/data/rust-mavlink-log.tlog").unwrap();
/// let mut stats = LinkStats::default();
/// for rec in TlogSlice::new(&bytes) {
///     stats.observe(rec.timestamp_us, &rec.frame);
/// }
/// let autopilot = stats.source(1, 1).unwrap();
/// assert_eq!(autopilot.lost, 0);
/// ```
#[derive(Debug, Clone, Default)]
pub struct LinkStats {
    sources: HashMap<(u8, u8), SourceStats>,
}

impl LinkStats {
    /// Records one frame received at `timestamp_us`.
    pub fn observe(&mut self, timestamp_us: u64, frame: &FrameRef<'_>) {
        let s = self
            .sources
            .entry((frame.sys_id(), frame.comp_id()))
            .or_insert_with(|| SourceStats {
                first_us: timestamp_us,
                ..SourceStats::default()
            });
        s.received += 1;
        s.last_us = s.last_us.max(timestamp_us);
        *s.by_msg_id.entry(frame.msg_id()).or_insert(0) += 1;
        let seq = frame.seq();
        match s.last_seq {
            None => s.last_seq = Some(seq),
            Some(prev) => {
                let step = seq.wrapping_sub(prev);
                if step == 0 {
                    s.duplicates += 1;
                } else if step <= 128 {
                    s.lost += step as u64 - 1;
                    s.last_seq = Some(seq);
                } else {
                    s.out_of_order += 1;
                }
            }
        }
    }

    /// Statistics for one sender.
    pub fn source(&self, sys_id: u8, comp_id: u8) -> Option<&SourceStats> {
        self.sources.get(&(sys_id, comp_id))
    }

    /// All senders, sorted by (system id, component id).
    pub fn sources(&self) -> Vec<((u8, u8), &SourceStats)> {
        let mut v: Vec<_> = self.sources.iter().map(|(k, s)| (*k, s)).collect();
        v.sort_by_key(|(k, _)| *k);
        v
    }

    /// Total frames received from all senders.
    pub fn total_received(&self) -> u64 {
        self.sources.values().map(|s| s.received).sum()
    }

    /// Total frames lost over all senders.
    pub fn total_lost(&self) -> u64 {
        self.sources.values().map(|s| s.lost).sum()
    }
}
