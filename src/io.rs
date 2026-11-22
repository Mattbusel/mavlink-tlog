//! Streaming reader and writer over `std::io` (needs the `std` feature).

use std::io::{self, Read, Write};

use crate::frame::FrameRef;
use crate::tlog::{
    note_invalid, try_record, Attempt, ReadOptions, ReadStats, Record, TIMESTAMP_LEN,
};

const CHUNK: usize = 64 * 1024;

/// An owned record, as returned by the [`TlogReader`] iterator.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OwnedRecord {
    /// Microseconds since the Unix epoch.
    pub timestamp_us: u64,
    /// The raw frame bytes.
    pub frame: Vec<u8>,
}

impl OwnedRecord {
    /// The frame, borrowed.
    pub fn frame(&self) -> FrameRef<'_> {
        FrameRef::from_bytes_unchecked(&self.frame)
    }
}

/// Reads records from any `Read` (a file, a pipe, a socket), with the same
/// resynchronization as [`crate::TlogSlice`]. Memory use is bounded by the
/// 64 KiB read buffer.
///
/// ```no_run
/// use mavlink_tlog::TlogReader;
///
/// let mut log = TlogReader::new(std::fs::File::open("flight.tlog")?);
/// while let Some(rec) = log.next_record()? {
///     println!("{} {:?} from {}/{}", rec.timestamp_us, rec.frame.message_name(), rec.frame.sys_id(), rec.frame.comp_id());
/// }
/// println!("{:?}", log.stats());
/// # Ok::<(), std::io::Error>(())
/// ```
#[derive(Debug)]
pub struct TlogReader<R> {
    inner: R,
    buf: Vec<u8>,
    start: usize,
    eof: bool,
    follow: bool,
    opts: ReadOptions,
    prev_ts: Option<u64>,
    stats: ReadStats,
}

impl<R: Read> TlogReader<R> {
    /// A reader with default options.
    pub fn new(inner: R) -> Self {
        Self::with_options(inner, ReadOptions::default())
    }

    /// A reader with the given options.
    pub fn with_options(inner: R, opts: ReadOptions) -> Self {
        TlogReader {
            inner,
            buf: Vec::with_capacity(CHUNK),
            start: 0,
            eof: false,
            follow: false,
            opts,
            prev_ts: None,
            stats: ReadStats::default(),
        }
    }

    /// A reader for a log that is still being written (a live capture
    /// from QGroundControl, MAVProxy or your own [`TlogWriter`]).
    ///
    /// At the end of the input, [`next_record`](Self::next_record) returns
    /// `Ok(None)` but keeps a partly written final record instead of
    /// counting it as damage, and the next call reads on from where the
    /// input ended. Poll it as the file grows; call
    /// [`finish`](Self::finish) when the writer is done to account for
    /// bytes that never became a record.
    ///
    /// ```no_run
    /// use mavlink_tlog::TlogReader;
    ///
    /// let mut log = TlogReader::follow(std::fs::File::open("live.tlog")?);
    /// loop {
    ///     while let Some(rec) = log.next_record()? {
    ///         println!("{} {:?}", rec.timestamp_us, rec.frame.message_name());
    ///     }
    ///     std::thread::sleep(std::time::Duration::from_millis(200));
    /// }
    /// # Ok::<(), std::io::Error>(())
    /// ```
    pub fn follow(inner: R) -> Self {
        Self::follow_with_options(inner, ReadOptions::default())
    }

    /// [`follow`](Self::follow) with the given options.
    pub fn follow_with_options(inner: R, opts: ReadOptions) -> Self {
        let mut r = Self::with_options(inner, opts);
        r.follow = true;
        r
    }

    /// Stops following: the input is complete. Later calls to
    /// [`next_record`](Self::next_record) read what is left and count a
    /// partial final record in [`ReadStats::truncated_tail`].
    pub fn finish(&mut self) {
        self.follow = false;
        self.eof = false;
    }

    /// `true` while the reader waits for more input at the end of a live
    /// log.
    pub fn is_following(&self) -> bool {
        self.follow
    }

    /// Bytes read from the input but not yet returned as a record: in
    /// follow mode, the partly written record at the end of the input.
    pub fn pending_bytes(&self) -> usize {
        self.buf.len() - self.start
    }

    /// What has been skipped so far.
    pub fn stats(&self) -> ReadStats {
        self.stats
    }

    /// The wrapped reader.
    pub fn into_inner(self) -> R {
        self.inner
    }

    fn fill(&mut self) -> io::Result<bool> {
        if self.eof {
            return Ok(false);
        }
        if self.start > 0 {
            self.buf.drain(..self.start);
            self.start = 0;
        }
        let old = self.buf.len();
        self.buf.resize(old + CHUNK, 0);
        let n = loop {
            match self.inner.read(&mut self.buf[old..]) {
                Ok(n) => break n,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    self.buf.truncate(old);
                    return Err(e);
                }
            }
        };
        self.buf.truncate(old + n);
        if n == 0 {
            self.eof = true;
        }
        Ok(n > 0)
    }

    /// The next record, borrowed from the reader's buffer.
    pub fn next_record(&mut self) -> io::Result<Option<Record<'_>>> {
        let (at, used, ts) = loop {
            match try_record(&self.buf[self.start..], &self.opts, self.prev_ts) {
                Attempt::Ok(rec, used) => break (self.start, used, rec.timestamp_us),
                Attempt::Incomplete => {
                    if self.fill()? {
                        continue;
                    }
                    if self.follow {
                        // The writer may not have finished this record yet:
                        // keep it and read on next time.
                        self.eof = false;
                        return Ok(None);
                    }
                    let rest = self.buf.len() - self.start;
                    if rest > TIMESTAMP_LEN + 1 {
                        self.start += 1;
                        self.stats.skipped_bytes += 1;
                        continue;
                    }
                    self.stats.truncated_tail += rest as u64;
                    self.start = self.buf.len();
                    return Ok(None);
                }
                Attempt::Invalid(e) => {
                    note_invalid(&mut self.stats, e);
                    self.start += 1;
                    self.stats.skipped_bytes += 1;
                }
            }
        };
        self.start = at + used;
        self.prev_ts = Some(ts);
        self.stats.records += 1;
        let frame = FrameRef::from_bytes_unchecked(&self.buf[at + TIMESTAMP_LEN..at + used]);
        Ok(Some(Record {
            timestamp_us: ts,
            frame,
        }))
    }
}

impl<R: Read> Iterator for TlogReader<R> {
    type Item = io::Result<OwnedRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.next_record() {
            Ok(Some(r)) => Some(Ok(OwnedRecord {
                timestamp_us: r.timestamp_us,
                frame: r.frame.as_bytes().to_vec(),
            })),
            Ok(None) => None,
            Err(e) => Some(Err(e)),
        }
    }
}

/// Writes records in the .tlog format.
///
/// ```
/// use mavlink_tlog::{TlogSlice, TlogWriter};
///
/// // HEARTBEAT, MAVLink 1, as encoded by pymavlink.
/// let heartbeat = [0xFE, 9, 7, 1, 1, 0, 4, 0, 0, 0, 2, 3, 0x51, 3, 3, 0x63, 0x91];
/// let mut w = TlogWriter::new(Vec::new());
/// w.write_frame(1_700_000_000_000_000, &heartbeat).unwrap();
/// let bytes = w.into_inner();
/// assert_eq!(bytes.len(), 8 + heartbeat.len());
/// assert_eq!(TlogSlice::new(&bytes).count(), 1);
/// ```
#[derive(Debug)]
pub struct TlogWriter<W> {
    inner: W,
    records: u64,
}

impl<W: Write> TlogWriter<W> {
    /// Wraps a writer. Wrap files in `BufWriter`; every record is two writes.
    pub fn new(inner: W) -> Self {
        TlogWriter { inner, records: 0 }
    }

    /// Writes one frame with the given timestamp. The bytes must be exactly
    /// one MAVLink frame (its length fields are checked, its checksum is not).
    pub fn write_frame(&mut self, timestamp_us: u64, frame: &[u8]) -> io::Result<()> {
        let parsed = FrameRef::parse_unchecked(frame)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        if parsed.len() != frame.len() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "bytes after the end of the frame",
            ));
        }
        self.inner.write_all(&timestamp_us.to_be_bytes())?;
        self.inner.write_all(frame)?;
        self.records += 1;
        Ok(())
    }

    /// Writes one frame stamped with the current system time.
    pub fn write_frame_now(&mut self, frame: &[u8]) -> io::Result<()> {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        self.write_frame(now, frame)
    }

    /// Records written so far.
    pub fn records(&self) -> u64 {
        self.records
    }

    /// Flushes the inner writer.
    pub fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }

    /// The wrapped writer.
    pub fn into_inner(self) -> W {
        self.inner
    }
}
