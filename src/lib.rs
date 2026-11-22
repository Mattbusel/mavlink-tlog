//! Read and write MAVLink `.tlog` telemetry logs.
//!
//! A `.tlog` is what QGroundControl, Mission Planner and MAVProxy save from
//! the telemetry link: every MAVLink frame received, each preceded by an
//! 8 byte big-endian timestamp in microseconds since the Unix epoch. This
//! crate reads them (from memory with [`TlogSlice`], or streaming from any
//! `Read` with [`TlogReader`]), writes them ([`TlogWriter`]), and measures
//! the link ([`LinkStats`]: frames, sequence-gap loss, duplicates,
//! message rates and the longest silence per sender) and checks MAVLink 2
//! signatures ([`SigningVerifier`]: forged, replayed and unsigned frames).
//! [`TlogReader::follow`] reads a log that is still being written.
//!
//! It works on raw frames and does not depend on a MAVLink dialect crate:
//! the checksum seed (CRC_EXTRA) of all 392 messages in MAVLink's `all.xml`
//! is built in, so damaged records are detected and skipped. Hand the frame
//! bytes to `mavlink` (rust-mavlink) or any other decoder to get typed
//! messages.
//!
//! ```
//! use mavlink_tlog::{LinkStats, TlogSlice};
//!
//! let bytes = std::fs::read("tests/data/rust-mavlink-log.tlog").unwrap();
//! let mut log = TlogSlice::new(&bytes);
//! let mut stats = LinkStats::default();
//! for rec in log.by_ref() {
//!     stats.observe(rec.timestamp_us, &rec.frame);
//! }
//! for ((sys, comp), s) in stats.sources() {
//!     println!("{sys}/{comp}: {} frames, {} lost", s.received, s.lost);
//! }
//! assert_eq!(log.stats().skipped_bytes, 0);
//! ```
//!
//! The core (frames, checksums, [`TlogSlice`]) is `no_std` and does not
//! allocate; turn off the default `std` feature to use it on its own.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod frame;
#[cfg(feature = "std")]
mod io;
pub mod signing;
#[cfg(feature = "std")]
mod stats;
mod table;
mod tlog;

pub use frame::{
    crc16, encode, message_info, CrcPolicy, FrameError, FrameRef, Header, MessageInfo, Version,
};
#[cfg(feature = "std")]
pub use io::{OwnedRecord, TlogReader, TlogWriter};
#[cfg(feature = "std")]
pub use signing::SigningVerifier;
pub use signing::{encode_signed, Signature, SignatureStatus, SIGNING_EPOCH_UNIX_US};
#[cfg(feature = "std")]
pub use stats::{LinkStats, SourceStats};
pub use tlog::{ReadOptions, ReadStats, Record, TlogSlice, TIMESTAMP_LEN};

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
struct ReadmeDoctests;

#[cfg(all(doctest, feature = "std"))]
#[doc = include_str!("../docs/integration.md")]
struct IntegrationDoctests;
