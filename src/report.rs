//! Was this telemetry log tampered with, and what does it say? One pass over
//! a `.tlog` that answers both questions for an investigator or a
//! chain-of-custody review (needs the `std` feature).
//!
//! [`FlightReport::from_bytes`] scans a log and returns:
//!
//! * an **integrity verdict** ([`Verdict`]) with the anomalies behind it, so
//!   you can say "intact", "intact and signed", "damaged between T1 and T2"
//!   or "shows signs of tampering between T1 and T2", and
//! * a concise **flight report**: first and last time, duration, every
//!   sender (system, component) with its frame count, rate, loss and worst
//!   dropout, and the messages it sent.
//!
//! The verdict is deliberate about what it claims. A logger writes frames in
//! the order it receives them, each stamped with the wall clock, so a record
//! whose time steps **backwards** did not come off the wire that way: the
//! file was reordered or spliced. A MAVLink 2 frame that passes its checksum
//! but fails its **signature** against the real key is a forgery or a replay,
//! not radio noise. Those two are reported as tampering. A **bad checksum**
//! or **unparsable bytes**, on the other hand, can be ordinary corruption or
//! a clumsy edit, so they are reported as damage, not proof of tampering, and
//! the counts are shown for you to judge. A segment deleted without breaking
//! time order looks like a dropout and is reported as a gap, never as proof.
//!
//! ```
//! use mavlink_tlog::{FlightReport, Verdict};
//!
//! let bytes = std::fs::read("tests/data/rust-mavlink-log.tlog").unwrap();
//! let report = FlightReport::from_bytes(&bytes);
//! assert!(report.records > 0);
//! assert!(!report.integrity.is_tampered());
//! // Print the whole thing for a human.
//! println!("{report}");
//! // Or act on the parts.
//! for s in &report.senders {
//!     println!("{}/{}: {} frames", s.sys_id, s.comp_id, s.received);
//! }
//! let _ = Verdict::Intact;
//! ```

use std::collections::BTreeMap;
use std::fmt;

use crate::frame::message_info;
use crate::signing::{SignatureStatus, SigningVerifier};
use crate::stats::LinkStats;
use crate::tlog::{ReadStats, TlogSlice};

/// How much the log can be trusted as a faithful record, strongest first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Verdict {
    /// No integrity anomalies, and every frame from a signing sender
    /// verified against the key. The strongest statement the tool can make.
    IntactSigned,
    /// No tamper or damage signals: every byte parsed into a frame whose
    /// checksum matched, time never went backwards, and (if a key was given)
    /// no signature failed. Not cryptographically proven unless a key made
    /// it [`IntactSigned`](Verdict::IntactSigned).
    Intact,
    /// Frames failed their checksum, or bytes would not parse. This is
    /// ordinary radio corruption or a clumsy edit; the tool cannot tell
    /// which from content alone. No reordering or signature forgery was
    /// found. See the counts.
    Damaged,
    /// Signs of tampering: time stepped backwards (reordered or spliced
    /// frames), a signature did not verify (a forgery), or a signed frame
    /// was replayed. These are not explained by corruption.
    Tampered,
}

impl Verdict {
    /// A short phrase for a one line summary.
    pub fn phrase(self) -> &'static str {
        match self {
            Verdict::IntactSigned => "intact and signed",
            Verdict::Intact => "intact",
            Verdict::Damaged => "damaged (corruption or edits)",
            Verdict::Tampered => "shows signs of tampering",
        }
    }
}

/// What kind of integrity anomaly was found.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AnomalyKind {
    /// A record's timestamp is older than the record before it: the log is
    /// not in the time order a logger writes, a sign of reordering or
    /// splicing. `magnitude` is how far time stepped back, in microseconds.
    /// A tamper signal.
    TimeRegression,
    /// Between two good records, bytes formed a frame whose checksum did not
    /// match its message definition, or a payload too long for its message:
    /// corruption or an edited payload. `magnitude` is how many such frames
    /// were found in the region. A damage signal.
    ChecksumFailure,
    /// Between two good records, bytes could not be read as frames and were
    /// skipped. Often benign (console text logged inline, a cut-off header);
    /// `magnitude` is the byte count. A damage signal.
    SkippedBytes,
    /// A signed frame did not verify against the key: a forgery, a different
    /// key, or damage the checksum did not catch. `magnitude` is 1. A tamper
    /// signal.
    BadSignature,
    /// A valid signature carrying a timestamp no newer than the last accepted
    /// on its stream: a replayed or duplicated signed frame. `magnitude` is
    /// 1. A tamper signal.
    Replay,
}

impl AnomalyKind {
    /// Whether this kind is a tamper signal (reordering or forgery), as
    /// opposed to damage (corruption).
    pub fn is_tamper(self) -> bool {
        matches!(
            self,
            AnomalyKind::TimeRegression | AnomalyKind::BadSignature | AnomalyKind::Replay
        )
    }
}

/// One integrity anomaly, anchored in time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Anomaly {
    /// What was found.
    pub kind: AnomalyKind,
    /// The timestamp this anomaly is anchored to: the record at or just
    /// after it, microseconds since the Unix epoch.
    pub at_us: u64,
    /// The previous good record's timestamp, microseconds. For a region
    /// anomaly the damage is between `prev_us` and `at_us`; for a time
    /// regression `at_us` is the older time and `prev_us` the newer one it
    /// went back from.
    pub prev_us: u64,
    /// A count whose meaning depends on [`kind`](Anomaly::kind): microseconds
    /// of backward step, frames, or bytes.
    pub magnitude: u64,
}

impl Anomaly {
    /// The time window this anomaly spans, `(low, high)` in microseconds.
    pub fn window(&self) -> (u64, u64) {
        (self.at_us.min(self.prev_us), self.at_us.max(self.prev_us))
    }
}

/// The integrity verdict and the evidence for it.
#[derive(Debug, Clone, Default)]
pub struct Integrity {
    /// Anomalies found, in the order they occur.
    pub anomalies: Vec<Anomaly>,
    /// Whether a signing key was supplied and signatures were checked.
    pub signing_checked: bool,
    /// Signed frames that verified against the key.
    pub signed_valid: u64,
    /// Signed frames that did not verify (forgeries).
    pub bad_signatures: u64,
    /// Signed frames replayed or duplicated.
    pub replays: u64,
    /// Unsigned frames from a sender that also sent signed frames: a gap in
    /// signing coverage (an unsigned command reaching a vehicle while
    /// signing was on). Not itself tampering of the log.
    pub unsigned_from_signers: u64,
    /// Bytes skipped because they would not parse (from [`ReadStats`]).
    pub skipped_bytes: u64,
    /// Frames whose checksum failed (from [`ReadStats`]).
    pub bad_crc: u64,
}

impl Integrity {
    /// The verdict these findings add up to.
    pub fn verdict(&self) -> Verdict {
        if self.anomalies.iter().any(|a| a.kind.is_tamper()) {
            Verdict::Tampered
        } else if self.bad_crc > 0 || self.skipped_bytes > 0 {
            Verdict::Damaged
        } else if self.signing_checked
            && self.signed_valid > 0
            && self.bad_signatures == 0
            && self.replays == 0
        {
            Verdict::IntactSigned
        } else {
            Verdict::Intact
        }
    }

    /// Whether any tamper signal (reordering or forgery) was found.
    pub fn is_tampered(&self) -> bool {
        self.anomalies.iter().any(|a| a.kind.is_tamper())
    }

    /// The time window `(from_us, to_us)` that tamper signals span, if any:
    /// the earliest and latest microsecond touched by a reordering or
    /// forgery anomaly. Lets you say "between T1 and T2".
    pub fn tamper_window(&self) -> Option<(u64, u64)> {
        let mut lo = u64::MAX;
        let mut hi = 0u64;
        for a in self.anomalies.iter().filter(|a| a.kind.is_tamper()) {
            let (l, h) = a.window();
            lo = lo.min(l);
            hi = hi.max(h);
        }
        (lo <= hi).then_some((lo, hi))
    }
}

/// One sender (system id, component id) and what it sent.
#[derive(Debug, Clone, PartialEq)]
pub struct SenderSummary {
    /// System id.
    pub sys_id: u8,
    /// Component id.
    pub comp_id: u8,
    /// Frames received.
    pub received: u64,
    /// Frames estimated lost from sequence gaps.
    pub lost: u64,
    /// Frames with a repeated sequence number.
    pub duplicates: u64,
    /// Frames whose sequence number went backwards.
    pub out_of_order: u64,
    /// First and last timestamps for this sender, microseconds.
    pub first_us: u64,
    /// Last timestamp, microseconds.
    pub last_us: u64,
    /// Average frame rate over this sender's span, Hz, if it can be computed.
    pub rate_hz: Option<f64>,
    /// Longest silence between two of this sender's frames, microseconds.
    pub longest_gap_us: u64,
    /// Timestamp that ended the longest silence, microseconds.
    pub longest_gap_end_us: u64,
    /// Frame count per message id, most frequent first.
    pub messages: Vec<(u32, u64)>,
}

impl SenderSummary {
    /// Loss as a fraction of frames that should have arrived.
    pub fn loss_ratio(&self) -> f64 {
        let total = self.received + self.lost;
        if total == 0 {
            0.0
        } else {
            self.lost as f64 / total as f64
        }
    }
}

/// The integrity verdict and flight report for one `.tlog`.
///
/// Build it with [`FlightReport::from_bytes`], or
/// [`FlightReport::from_bytes_with_key`] to also verify MAVLink 2 signatures.
#[derive(Debug, Clone)]
pub struct FlightReport {
    /// Frames read.
    pub records: u64,
    /// First record timestamp, microseconds since the Unix epoch.
    pub first_us: Option<u64>,
    /// Last record timestamp, microseconds since the Unix epoch.
    pub last_us: Option<u64>,
    /// Longest silence anywhere in the log (no frame from any sender),
    /// microseconds, and the timestamp that ended it.
    pub longest_silence_us: u64,
    /// Timestamp that ended the longest whole-log silence, microseconds.
    pub longest_silence_end_us: u64,
    /// Every sender seen, sorted by (system id, component id).
    pub senders: Vec<SenderSummary>,
    /// What the reader had to skip (bytes, bad checksums, truncated tail).
    pub read: ReadStats,
    /// The integrity verdict and its evidence.
    pub integrity: Integrity,
}

impl FlightReport {
    /// Scans a `.tlog` held in memory and builds the report. Does not check
    /// signatures; use [`from_bytes_with_key`](Self::from_bytes_with_key)
    /// for that.
    pub fn from_bytes(bytes: &[u8]) -> FlightReport {
        Self::build(bytes, None)
    }

    /// Scans a `.tlog` and also verifies every MAVLink 2 signature against
    /// `key`, so the verdict can reach [`Verdict::IntactSigned`] or flag
    /// forgeries and replays.
    pub fn from_bytes_with_key(bytes: &[u8], key: [u8; 32]) -> FlightReport {
        Self::build(bytes, Some(key))
    }

    /// Duration from the first to the last record, seconds.
    pub fn duration_s(&self) -> Option<f64> {
        let (a, b) = (self.first_us?, self.last_us?);
        Some(b.saturating_sub(a) as f64 / 1e6)
    }

    /// The overall frame rate across the whole log, Hz.
    pub fn rate_hz(&self) -> Option<f64> {
        let span = self.duration_s()?;
        if span <= 0.0 || self.records < 2 {
            return None;
        }
        Some((self.records - 1) as f64 / span)
    }

    /// The verdict, for convenience.
    pub fn verdict(&self) -> Verdict {
        self.integrity.verdict()
    }

    fn build(bytes: &[u8], key: Option<[u8; 32]>) -> FlightReport {
        let mut log = TlogSlice::new(bytes);
        let mut stats = LinkStats::default();
        let mut verifier = key.map(SigningVerifier::new);

        let mut anomalies = Vec::new();
        let mut signed_valid = 0u64;
        let mut bad_signatures = 0u64;
        let mut replays = 0u64;
        // Per sender: (frames that were signed, frames that were unsigned).
        let mut signing_seen: BTreeMap<(u8, u8), (u64, u64)> = BTreeMap::new();

        let mut prev_read = ReadStats::default();
        let mut prev_ts: Option<u64> = None;
        let mut first_us = None;
        let mut last_us = None;
        let mut longest_silence_us = 0u64;
        let mut longest_silence_end_us = 0u64;

        while let Some(rec) = log.next() {
            let now = log.stats();
            let ts = rec.timestamp_us;
            let anchor = prev_ts.unwrap_or(ts);

            // Damage in the bytes between the previous record and this one.
            let new_bad = now.bad_crc - prev_read.bad_crc;
            let new_skipped = now.skipped_bytes - prev_read.skipped_bytes;
            if new_bad > 0 {
                anomalies.push(Anomaly {
                    kind: AnomalyKind::ChecksumFailure,
                    at_us: ts,
                    prev_us: anchor,
                    magnitude: new_bad,
                });
            }
            if new_skipped > 0 {
                anomalies.push(Anomaly {
                    kind: AnomalyKind::SkippedBytes,
                    at_us: ts,
                    prev_us: anchor,
                    magnitude: new_skipped,
                });
            }

            // Time order: a logger writes in receive order, so a step back
            // means the file was reordered or spliced.
            if let Some(p) = prev_ts {
                if ts < p {
                    anomalies.push(Anomaly {
                        kind: AnomalyKind::TimeRegression,
                        at_us: ts,
                        prev_us: p,
                        magnitude: p - ts,
                    });
                } else {
                    let gap = ts - p;
                    if gap > longest_silence_us {
                        longest_silence_us = gap;
                        longest_silence_end_us = ts;
                    }
                }
            }

            // Signatures.
            if let Some(v) = verifier.as_mut() {
                let key = (rec.frame.sys_id(), rec.frame.comp_id());
                match v.check(&rec.frame) {
                    SignatureStatus::Valid => {
                        signed_valid += 1;
                        signing_seen.entry(key).or_default().0 += 1;
                    }
                    SignatureStatus::BadSignature => {
                        bad_signatures += 1;
                        signing_seen.entry(key).or_default().0 += 1;
                        anomalies.push(Anomaly {
                            kind: AnomalyKind::BadSignature,
                            at_us: ts,
                            prev_us: ts,
                            magnitude: 1,
                        });
                    }
                    SignatureStatus::Replay => {
                        replays += 1;
                        signing_seen.entry(key).or_default().0 += 1;
                        anomalies.push(Anomaly {
                            kind: AnomalyKind::Replay,
                            at_us: ts,
                            prev_us: ts,
                            magnitude: 1,
                        });
                    }
                    SignatureStatus::Unsigned => {
                        signing_seen.entry(key).or_default().1 += 1;
                    }
                }
            }

            stats.observe(ts, &rec.frame);
            first_us.get_or_insert(ts);
            last_us = Some(last_us.map_or(ts, |l: u64| l.max(ts)));
            prev_ts = Some(ts);
            prev_read = now;
        }

        let read = log.stats();
        let unsigned_from_signers = signing_seen
            .values()
            .filter(|(signed, _)| *signed > 0)
            .map(|(_, unsigned)| *unsigned)
            .sum();

        let senders = stats
            .sources()
            .into_iter()
            .map(|((sys, comp), s)| {
                let mut messages: Vec<(u32, u64)> =
                    s.by_msg_id.iter().map(|(id, n)| (*id, *n)).collect();
                messages.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
                SenderSummary {
                    sys_id: sys,
                    comp_id: comp,
                    received: s.received,
                    lost: s.lost,
                    duplicates: s.duplicates,
                    out_of_order: s.out_of_order,
                    first_us: s.first_us,
                    last_us: s.last_us,
                    rate_hz: s.rate_total_hz(),
                    longest_gap_us: s.longest_gap_us,
                    longest_gap_end_us: s.longest_gap_end_us,
                    messages,
                }
            })
            .collect();

        let integrity = Integrity {
            anomalies,
            signing_checked: key.is_some(),
            signed_valid,
            bad_signatures,
            replays,
            unsigned_from_signers,
            skipped_bytes: read.skipped_bytes,
            bad_crc: read.bad_crc,
        };

        FlightReport {
            records: read.records,
            first_us,
            last_us,
            longest_silence_us,
            longest_silence_end_us,
            senders,
            read,
            integrity,
        }
    }
}

fn msg_name(id: u32) -> String {
    message_info(id).map_or_else(|| format!("UNKNOWN_{id}"), |i| i.name.to_string())
}

impl fmt::Display for FlightReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let verdict = self.verdict();
        let first = self.first_us.unwrap_or(0);

        // Headline integrity statement.
        write!(f, "INTEGRITY: {}", verdict.phrase())?;
        if let Some((lo, hi)) = self.integrity.tamper_window() {
            let a = (lo.saturating_sub(first)) as f64 / 1e6;
            let b = (hi.saturating_sub(first)) as f64 / 1e6;
            if (b - a).abs() < 1e-6 {
                write!(f, " at +{a:.3} s (Unix us {lo})")?;
            } else {
                write!(f, " between +{a:.3} s and +{b:.3} s (Unix us {lo}..{hi})")?;
            }
        }
        writeln!(f)?;
        if self.integrity.bad_crc > 0 || self.integrity.skipped_bytes > 0 {
            writeln!(
                f,
                "  {} frames failed their checksum, {} bytes would not parse",
                self.integrity.bad_crc, self.integrity.skipped_bytes
            )?;
        }
        if self.read.truncated_tail > 0 {
            writeln!(
                f,
                "  {} bytes at the end did not form a record (log may be cut short)",
                self.read.truncated_tail
            )?;
        }
        if self.integrity.signing_checked {
            writeln!(
                f,
                "  signatures: {} valid, {} forged, {} replayed, {} unsigned from signing senders",
                self.integrity.signed_valid,
                self.integrity.bad_signatures,
                self.integrity.replays,
                self.integrity.unsigned_from_signers
            )?;
        }
        for a in &self.integrity.anomalies {
            let rel = (a.at_us.saturating_sub(first)) as f64 / 1e6;
            match a.kind {
                AnomalyKind::TimeRegression => writeln!(
                    f,
                    "  time went back {:.3} s at +{rel:.3} s (reordered or spliced)",
                    a.magnitude as f64 / 1e6
                )?,
                AnomalyKind::ChecksumFailure => writeln!(
                    f,
                    "  {} bad-checksum frame(s) near +{rel:.3} s",
                    a.magnitude
                )?,
                AnomalyKind::SkippedBytes => {
                    writeln!(f, "  {} unparsable byte(s) near +{rel:.3} s", a.magnitude)?
                }
                AnomalyKind::BadSignature => writeln!(f, "  forged signature at +{rel:.3} s")?,
                AnomalyKind::Replay => writeln!(f, "  replayed signed frame at +{rel:.3} s")?,
            }
        }

        // Flight report.
        writeln!(f, "\nFLIGHT REPORT")?;
        writeln!(f, "  frames: {}", self.records)?;
        match (self.first_us, self.last_us) {
            (Some(a), Some(b)) => {
                writeln!(f, "  start:  Unix us {a}")?;
                writeln!(f, "  end:    Unix us {b}")?;
                if let Some(d) = self.duration_s() {
                    let rate = self
                        .rate_hz()
                        .map_or_else(|| "-".to_string(), |r| format!("{r:.1} Hz"));
                    writeln!(f, "  span:   {d:.1} s, {rate} overall")?;
                }
            }
            _ => writeln!(f, "  (no records)")?,
        }
        if self.longest_silence_us > 0 {
            writeln!(
                f,
                "  longest whole-log silence: {:.3} s, ending at +{:.3} s",
                self.longest_silence_us as f64 / 1e6,
                (self.longest_silence_end_us.saturating_sub(first)) as f64 / 1e6
            )?;
        }
        writeln!(f, "  systems seen: {}", self.senders.len())?;
        for s in &self.senders {
            let secs = s.last_us.saturating_sub(s.first_us) as f64 / 1e6;
            let rate = s
                .rate_hz
                .map_or_else(|| "-".to_string(), |r| format!("{r:.2} Hz"));
            writeln!(
                f,
                "\n  sender {}/{}: {} frames over {secs:.1} s, {rate}, {} lost ({:.2}%), {} duplicates, {} out of order",
                s.sys_id,
                s.comp_id,
                s.received,
                s.lost,
                s.loss_ratio() * 100.0,
                s.duplicates,
                s.out_of_order
            )?;
            if s.longest_gap_us > 0 {
                writeln!(
                    f,
                    "    longest silence {:.3} s, ending at +{:.3} s",
                    s.longest_gap_us as f64 / 1e6,
                    (s.longest_gap_end_us.saturating_sub(first)) as f64 / 1e6
                )?;
            }
            for (id, n) in &s.messages {
                writeln!(f, "    {:<32} {n:>9}", msg_name(*id))?;
            }
        }
        Ok(())
    }
}
