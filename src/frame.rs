//! Raw MAVLink v1 / v2 frames: boundaries, header fields and checksum.

use crate::table::MESSAGES;

/// Start byte of a MAVLink 1 frame.
pub const MAGIC_V1: u8 = 0xFE;
/// Start byte of a MAVLink 2 frame.
pub const MAGIC_V2: u8 = 0xFD;
/// Incompatibility flag: the frame carries a 13 byte signature.
pub const IFLAG_SIGNED: u8 = 0x01;
/// Length of a MAVLink 2 signature.
pub const SIGNATURE_LEN: usize = 13;

/// Information about a message id from the built-in table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MessageInfo {
    /// Message id.
    pub id: u32,
    /// CRC_EXTRA seed byte.
    pub crc_extra: u8,
    /// Payload length with all extension fields.
    pub max_len: u8,
    /// Message name, for example `"HEARTBEAT"`.
    pub name: &'static str,
}

/// Looks up a message id in the built-in table (all messages of the
/// MAVLink `all.xml` dialect: common, ardupilotmega, development and the
/// vendor sets).
pub fn message_info(id: u32) -> Option<MessageInfo> {
    MESSAGES.binary_search_by_key(&id, |r| r.0).ok().map(|i| {
        let (id, crc_extra, max_len, name) = MESSAGES[i];
        MessageInfo {
            id,
            crc_extra,
            max_len,
            name,
        }
    })
}

/// CRC-16/MCRF4XX (the X.25 variant MAVLink uses), one byte at a time.
pub fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let mut tmp = byte ^ (crc & 0xFF) as u8;
    tmp ^= tmp << 4;
    (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4)
}

/// CRC-16/MCRF4XX over `data`, starting from 0xFFFF.
pub fn crc16(data: &[u8]) -> u16 {
    data.iter().fold(0xFFFF, |crc, &b| crc_accumulate(b, crc))
}

/// MAVLink protocol version of a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Version {
    /// MAVLink 1 (start byte 0xFE).
    V1,
    /// MAVLink 2 (start byte 0xFD).
    V2,
}

/// Why bytes could not be read as a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// More bytes are needed; the frame would be this long.
    Incomplete {
        /// Total length the frame needs (or the header, when unknown yet).
        need: usize,
    },
    /// The first byte is not a MAVLink start byte.
    BadMagic,
    /// A MAVLink 2 frame sets incompatibility flags this crate does not know.
    UnknownIncompatFlags(u8),
    /// The checksum does not match.
    BadCrc {
        /// Checksum in the frame.
        found: u16,
        /// Checksum computed with the message's CRC_EXTRA.
        expected: u16,
    },
    /// The message id is not in the table and the policy requires it.
    UnknownMessage(u32),
    /// The payload is longer than the message allows.
    PayloadTooLong {
        /// Message id.
        id: u32,
        /// Length found.
        len: u8,
    },
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            FrameError::Incomplete { need } => write!(f, "incomplete frame, need {need} bytes"),
            FrameError::BadMagic => f.write_str("not a MAVLink start byte"),
            FrameError::UnknownIncompatFlags(v) => {
                write!(f, "unknown incompatibility flags {v:#04x}")
            }
            FrameError::BadCrc { found, expected } => {
                write!(f, "bad checksum {found:#06x}, expected {expected:#06x}")
            }
            FrameError::UnknownMessage(id) => write!(f, "unknown message id {id}"),
            FrameError::PayloadTooLong { id, len } => {
                write!(f, "payload of {len} bytes is too long for message {id}")
            }
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for FrameError {}

/// How strictly frames are checked.
#[derive(Debug, Clone, Copy, Default)]
pub enum CrcPolicy {
    /// Verify the checksum of messages in the table; accept other message
    /// ids without a checksum check (useful with custom dialects).
    #[default]
    Lenient,
    /// Verify every checksum and reject message ids not in the table. This
    /// matches pymavlink with the `all` dialect.
    Strict,
    /// Check nothing beyond the frame structure.
    Off,
    /// Use your own CRC_EXTRA lookup (for a private dialect). Ids for which
    /// it returns `None` are rejected.
    Custom(fn(u32) -> Option<u8>),
}

/// A borrowed MAVLink frame, from start byte to the end of its checksum
/// (or signature).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FrameRef<'a> {
    bytes: &'a [u8],
}

impl<'a> FrameRef<'a> {
    /// Finds the frame at the start of `data` and checks it under `policy`.
    /// Bytes after the frame are ignored.
    pub fn parse(data: &'a [u8], policy: CrcPolicy) -> Result<Self, FrameError> {
        let frame = Self::parse_unchecked(data)?;
        frame.check(policy)?;
        Ok(frame)
    }

    /// Finds the frame boundaries without checking the checksum.
    pub fn parse_unchecked(data: &'a [u8]) -> Result<Self, FrameError> {
        let magic = *data.first().ok_or(FrameError::Incomplete { need: 1 })?;
        let total = match magic {
            MAGIC_V1 => {
                let len = *data.get(1).ok_or(FrameError::Incomplete { need: 8 })? as usize;
                6 + len + 2
            }
            MAGIC_V2 => {
                if data.len() < 3 {
                    return Err(FrameError::Incomplete { need: 12 });
                }
                let len = data[1] as usize;
                let flags = data[2];
                if flags & !IFLAG_SIGNED != 0 {
                    return Err(FrameError::UnknownIncompatFlags(flags));
                }
                10 + len
                    + 2
                    + if flags & IFLAG_SIGNED != 0 {
                        SIGNATURE_LEN
                    } else {
                        0
                    }
            }
            _ => return Err(FrameError::BadMagic),
        };
        if data.len() < total {
            return Err(FrameError::Incomplete { need: total });
        }
        Ok(FrameRef {
            bytes: &data[..total],
        })
    }

    /// Checks the checksum and message id under `policy`.
    pub fn check(&self, policy: CrcPolicy) -> Result<(), FrameError> {
        let id = self.msg_id();
        let extra = match policy {
            CrcPolicy::Off => return Ok(()),
            CrcPolicy::Custom(f) => f(id).ok_or(FrameError::UnknownMessage(id))?,
            CrcPolicy::Lenient | CrcPolicy::Strict => match message_info(id) {
                Some(info) => {
                    if self.payload_len() > info.max_len {
                        return Err(FrameError::PayloadTooLong {
                            id,
                            len: self.payload_len(),
                        });
                    }
                    info.crc_extra
                }
                None if matches!(policy, CrcPolicy::Lenient) => return Ok(()),
                None => return Err(FrameError::UnknownMessage(id)),
            },
        };
        let expected = self.compute_crc(extra);
        let found = self.checksum();
        if expected != found {
            return Err(FrameError::BadCrc { found, expected });
        }
        Ok(())
    }

    /// Wraps bytes you already know form exactly one frame.
    pub fn from_bytes_unchecked(bytes: &'a [u8]) -> Self {
        FrameRef { bytes }
    }

    /// All bytes of the frame.
    pub fn as_bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// Protocol version.
    pub fn version(&self) -> Version {
        if self.bytes[0] == MAGIC_V2 {
            Version::V2
        } else {
            Version::V1
        }
    }

    fn header_len(&self) -> usize {
        match self.version() {
            Version::V1 => 6,
            Version::V2 => 10,
        }
    }

    /// Payload length as sent (MAVLink 2 trims trailing zero bytes).
    pub fn payload_len(&self) -> u8 {
        self.bytes[1]
    }

    /// Incompatibility flags (0 for MAVLink 1).
    pub fn incompat_flags(&self) -> u8 {
        if self.version() == Version::V2 {
            self.bytes[2]
        } else {
            0
        }
    }

    /// Compatibility flags (0 for MAVLink 1).
    pub fn compat_flags(&self) -> u8 {
        if self.version() == Version::V2 {
            self.bytes[3]
        } else {
            0
        }
    }

    /// Packet sequence number.
    pub fn seq(&self) -> u8 {
        self.bytes[self.seq_index()]
    }

    /// Sender system id.
    pub fn sys_id(&self) -> u8 {
        self.bytes[self.seq_index() + 1]
    }

    /// Sender component id.
    pub fn comp_id(&self) -> u8 {
        self.bytes[self.seq_index() + 2]
    }

    fn seq_index(&self) -> usize {
        match self.version() {
            Version::V1 => 2,
            Version::V2 => 4,
        }
    }

    /// Message id.
    pub fn msg_id(&self) -> u32 {
        match self.version() {
            Version::V1 => self.bytes[5] as u32,
            Version::V2 => u32::from_le_bytes([self.bytes[7], self.bytes[8], self.bytes[9], 0]),
        }
    }

    /// Message name from the built-in table.
    pub fn message_name(&self) -> Option<&'static str> {
        message_info(self.msg_id()).map(|i| i.name)
    }

    /// The payload as sent.
    pub fn payload(&self) -> &'a [u8] {
        let h = self.header_len();
        &self.bytes[h..h + self.payload_len() as usize]
    }

    /// Copies the payload into `out` and zero-fills the rest, restoring the
    /// trailing zeros MAVLink 2 trims. Returns the full length written
    /// (the message's maximum length when it is in the table).
    pub fn payload_padded(&self, out: &mut [u8; 255]) -> usize {
        let p = self.payload();
        out.fill(0);
        out[..p.len()].copy_from_slice(p);
        message_info(self.msg_id()).map_or(p.len(), |i| (i.max_len as usize).max(p.len()))
    }

    /// The checksum in the frame.
    pub fn checksum(&self) -> u16 {
        let at = self.header_len() + self.payload_len() as usize;
        u16::from_le_bytes([self.bytes[at], self.bytes[at + 1]])
    }

    /// The checksum computed with `crc_extra`.
    pub fn compute_crc(&self, crc_extra: u8) -> u16 {
        let end = self.header_len() + self.payload_len() as usize;
        crc_accumulate(crc_extra, crc16(&self.bytes[1..end]))
    }

    /// The 13 byte signature, if the frame is signed.
    pub fn signature(&self) -> Option<&'a [u8]> {
        if self.incompat_flags() & IFLAG_SIGNED != 0 {
            Some(&self.bytes[self.bytes.len() - SIGNATURE_LEN..])
        } else {
            None
        }
    }

    /// Total length of the frame in bytes.
    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    /// Always `false`: a frame has at least a header and checksum.
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

/// Header fields for [`encode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Header {
    /// Packet sequence number.
    pub seq: u8,
    /// Sender system id.
    pub sys_id: u8,
    /// Sender component id.
    pub comp_id: u8,
    /// Message id (MAVLink 1 only carries ids up to 255).
    pub msg_id: u32,
}

/// Writes an unsigned frame into `out` and returns its length. MAVLink 2
/// frames get their trailing zero payload bytes trimmed, as the protocol
/// requires. `crc_extra` of `None` looks the message up in the built-in
/// table.
pub fn encode(
    version: Version,
    header: Header,
    payload: &[u8],
    crc_extra: Option<u8>,
    out: &mut [u8],
) -> Option<usize> {
    let extra = match crc_extra {
        Some(e) => e,
        None => message_info(header.msg_id)?.crc_extra,
    };
    let payload = match version {
        Version::V1 => payload,
        Version::V2 => {
            let keep = payload.iter().rposition(|&b| b != 0).map_or(1, |i| i + 1);
            &payload[..keep.min(payload.len())]
        }
    };
    if payload.len() > 255 {
        return None;
    }
    let h = match version {
        Version::V1 => {
            if header.msg_id > 255 {
                return None;
            }
            6
        }
        Version::V2 => 10,
    };
    let total = h + payload.len() + 2;
    if out.len() < total || header.msg_id > 0xFF_FFFF {
        return None;
    }
    match version {
        Version::V1 => {
            out[..6].copy_from_slice(&[
                MAGIC_V1,
                payload.len() as u8,
                header.seq,
                header.sys_id,
                header.comp_id,
                header.msg_id as u8,
            ]);
        }
        Version::V2 => {
            let id = header.msg_id.to_le_bytes();
            out[..10].copy_from_slice(&[
                MAGIC_V2,
                payload.len() as u8,
                0,
                0,
                header.seq,
                header.sys_id,
                header.comp_id,
                id[0],
                id[1],
                id[2],
            ]);
        }
    }
    out[h..h + payload.len()].copy_from_slice(payload);
    let crc = crc_accumulate(extra, crc16(&out[1..h + payload.len()]));
    out[h + payload.len()..total].copy_from_slice(&crc.to_le_bytes());
    Some(total)
}
