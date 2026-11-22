//! MAVLink 2 message signing: read, verify and create signatures.
//!
//! A signed MAVLink 2 frame sets incompatibility flag 0x01 and ends with 13
//! bytes: a link id (1 byte), a timestamp (6 bytes, little-endian, units of
//! 10 microseconds since 2015-01-01 00:00:00 UTC) and the first 6 bytes of
//! `SHA-256(secret_key || frame from start byte through link id and
//! timestamp)`. A receiver accepts a frame when the signature matches and
//! the timestamp is newer than the last one it accepted on the same
//! (system, component, link id) stream; anything else is a forgery or a
//! replay.
//!
//! This module is `no_std` and does not allocate, except
//! [`SigningVerifier`], which tracks timestamps per stream and needs `std`.
//!
//! ```
//! use mavlink_tlog::{encode_signed, FrameRef, CrcPolicy, Header};
//!
//! let key = [7u8; 32];
//! let mut buf = [0u8; 280];
//! let hb = Header { seq: 0, sys_id: 1, comp_id: 1, msg_id: 0 };
//! let n = encode_signed(hb, &[0, 0, 0, 0, 2, 3, 0x51, 3, 3], None, &key, 0, 1_000, &mut buf).unwrap();
//! let frame = FrameRef::parse(&buf[..n], CrcPolicy::Strict).unwrap();
//! assert!(frame.verify_signature(&key));
//! assert!(!frame.verify_signature(&[8u8; 32]));
//! assert_eq!(frame.signature_fields().unwrap().timestamp, 1_000);
//! ```

use crate::frame::{
    crc16, crc_accumulate, encode, message_info, FrameRef, Header, Version, IFLAG_SIGNED,
    SIGNATURE_LEN,
};

/// Microseconds from the Unix epoch to 2015-01-01 00:00:00 UTC, the epoch
/// of MAVLink signing timestamps.
pub const SIGNING_EPOCH_UNIX_US: u64 = 1_420_070_400_000_000;

/// The parts of a MAVLink 2 signature.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Signature {
    /// Link id the sender signed for.
    pub link_id: u8,
    /// Signing timestamp in units of 10 microseconds since 2015-01-01 UTC
    /// (48 bits).
    pub timestamp: u64,
    /// The 6 byte truncated SHA-256.
    pub value: [u8; 6],
}

impl Signature {
    /// The signing timestamp as microseconds since the Unix epoch, for
    /// comparison with the tlog record time.
    pub fn unix_us(&self) -> u64 {
        SIGNING_EPOCH_UNIX_US + self.timestamp * 10
    }
}

impl<'a> FrameRef<'a> {
    /// Link id, timestamp and value of the signature, if the frame is
    /// signed.
    pub fn signature_fields(&self) -> Option<Signature> {
        let s = self.signature()?;
        let mut ts = [0u8; 8];
        ts[..6].copy_from_slice(&s[1..7]);
        let mut value = [0u8; 6];
        value.copy_from_slice(&s[7..13]);
        Some(Signature {
            link_id: s[0],
            timestamp: u64::from_le_bytes(ts),
            value,
        })
    }

    /// `true` when the frame is signed and its signature matches `key`.
    /// This checks the bytes only; replay protection needs the timestamp
    /// history that [`SigningVerifier`] keeps.
    pub fn verify_signature(&self, key: &[u8; 32]) -> bool {
        match self.signature() {
            Some(sig) => {
                let bytes = self.as_bytes();
                let signed = &bytes[..bytes.len() - 6];
                signature_value(key, signed) == sig[7..13]
            }
            None => false,
        }
    }
}

fn signature_value(key: &[u8; 32], signed: &[u8]) -> [u8; 6] {
    let mut h = Sha256::new();
    h.update(key);
    h.update(signed);
    let d = h.finish();
    let mut out = [0u8; 6];
    out.copy_from_slice(&d[..6]);
    out
}

/// Writes a signed MAVLink 2 frame into `out` and returns its length.
/// `timestamp` is in units of 10 microseconds since 2015-01-01 UTC and must
/// fit in 48 bits. `crc_extra` of `None` looks the message up in the
/// built-in table. Returns `None` when the message is unknown, `out` is too
/// short or the timestamp is too large.
pub fn encode_signed(
    header: Header,
    payload: &[u8],
    crc_extra: Option<u8>,
    key: &[u8; 32],
    link_id: u8,
    timestamp: u64,
    out: &mut [u8],
) -> Option<usize> {
    if timestamp >= 1 << 48 {
        return None;
    }
    let extra = match crc_extra {
        Some(e) => e,
        None => message_info(header.msg_id)?.crc_extra,
    };
    let n = encode(Version::V2, header, payload, Some(extra), out)?;
    let total = n + SIGNATURE_LEN;
    if out.len() < total {
        return None;
    }
    out[2] |= IFLAG_SIGNED;
    let crc = crc_accumulate(extra, crc16(&out[1..n - 2]));
    out[n - 2..n].copy_from_slice(&crc.to_le_bytes());
    out[n] = link_id;
    out[n + 1..n + 7].copy_from_slice(&timestamp.to_le_bytes()[..6]);
    let value = signature_value(key, &out[..n + 7]);
    out[n + 7..total].copy_from_slice(&value);
    Some(total)
}

/// What [`SigningVerifier::check`] found.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SignatureStatus {
    /// The frame carries no signature.
    Unsigned,
    /// The signature matches and the timestamp is newer than any accepted
    /// before on its stream.
    Valid,
    /// The signature does not match the key: a forgery, a different key,
    /// or damage the checksum did not catch.
    BadSignature,
    /// The signature matches but the timestamp is not newer than the last
    /// one accepted on the same (system, component, link id) stream: the
    /// frame is a replay or a duplicate.
    Replay,
}

/// Verifies signatures with one key and rejects replays, as a MAVLink 2
/// receiver does: a frame is accepted only when its timestamp is newer than
/// the last accepted one on the same (system, component, link id) stream.
///
/// ```
/// use mavlink_tlog::{encode_signed, FrameRef, Header, SignatureStatus, SigningVerifier};
///
/// let key = [7u8; 32];
/// let mut buf = [0u8; 280];
/// let hb = Header { seq: 0, sys_id: 1, comp_id: 1, msg_id: 0 };
/// let n = encode_signed(hb, &[0, 0, 0, 0, 2, 3, 0x51, 3, 3], None, &key, 0, 1_000, &mut buf).unwrap();
/// let frame = FrameRef::from_bytes_unchecked(&buf[..n]);
///
/// let mut v = SigningVerifier::new(key);
/// assert_eq!(v.check(&frame), SignatureStatus::Valid);
/// assert_eq!(v.check(&frame), SignatureStatus::Replay); // same timestamp again
/// ```
#[cfg(feature = "std")]
#[derive(Debug, Clone)]
pub struct SigningVerifier {
    key: [u8; 32],
    last: std::collections::HashMap<(u8, u8, u8), u64>,
}

#[cfg(feature = "std")]
impl SigningVerifier {
    /// A verifier for `key`, with no history.
    pub fn new(key: [u8; 32]) -> Self {
        SigningVerifier {
            key,
            last: std::collections::HashMap::new(),
        }
    }

    /// Checks one frame and, when it is valid, remembers its timestamp.
    pub fn check(&mut self, frame: &FrameRef<'_>) -> SignatureStatus {
        let Some(sig) = frame.signature_fields() else {
            return SignatureStatus::Unsigned;
        };
        if !frame.verify_signature(&self.key) {
            return SignatureStatus::BadSignature;
        }
        let stream = (frame.sys_id(), frame.comp_id(), sig.link_id);
        match self.last.get(&stream) {
            Some(&prev) if sig.timestamp <= prev => SignatureStatus::Replay,
            _ => {
                self.last.insert(stream, sig.timestamp);
                SignatureStatus::Valid
            }
        }
    }

    /// The last accepted timestamp on a (system, component, link id)
    /// stream.
    pub fn last_timestamp(&self, sys_id: u8, comp_id: u8, link_id: u8) -> Option<u64> {
        self.last.get(&(sys_id, comp_id, link_id)).copied()
    }
}

/// Minimal SHA-256 (FIPS 180-4), enough for MAVLink signing.
struct Sha256 {
    state: [u32; 8],
    block: [u8; 64],
    fill: usize,
    total: u64,
}

const K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

impl Sha256 {
    fn new() -> Self {
        Sha256 {
            state: [
                0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
                0x5be0cd19,
            ],
            block: [0; 64],
            fill: 0,
            total: 0,
        }
    }

    fn update(&mut self, mut data: &[u8]) {
        self.total += data.len() as u64;
        while !data.is_empty() {
            let take = (64 - self.fill).min(data.len());
            self.block[self.fill..self.fill + take].copy_from_slice(&data[..take]);
            self.fill += take;
            data = &data[take..];
            if self.fill == 64 {
                let block = self.block;
                self.compress(&block);
                self.fill = 0;
            }
        }
    }

    fn finish(mut self) -> [u8; 32] {
        let bits = self.total.wrapping_mul(8);
        self.update(&[0x80]);
        while self.fill != 56 {
            self.update(&[0]);
        }
        self.update(&bits.to_be_bytes());
        let mut out = [0u8; 32];
        for (o, s) in out.chunks_exact_mut(4).zip(self.state) {
            o.copy_from_slice(&s.to_be_bytes());
        }
        out
    }

    fn compress(&mut self, block: &[u8; 64]) {
        let mut w = [0u32; 64];
        for (i, c) in block.chunks_exact(4).enumerate() {
            w[i] = u32::from_be_bytes([c[0], c[1], c[2], c[3]]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = self.state;
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ (!e & g);
            let t1 = h
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        for (s, v) in self.state.iter_mut().zip([a, b, c, d, e, f, g, h]) {
            *s = s.wrapping_add(v);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sha(data: &[u8]) -> [u8; 32] {
        let mut h = Sha256::new();
        h.update(data);
        h.finish()
    }

    #[test]
    fn sha256_fips_vectors() {
        assert_eq!(
            sha(b"abc")[..8],
            [0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea]
        );
        assert_eq!(
            sha(b"")[..8],
            [0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14]
        );
        // 448 bit message: padding spills into a second block.
        assert_eq!(
            sha(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")[28..],
            [0x19, 0xdb, 0x06, 0xc1]
        );
        // One million 'a', fed in uneven pieces.
        let mut h = Sha256::new();
        let chunk = [b'a'; 997];
        let mut left = 1_000_000usize;
        while left > 0 {
            let n = left.min(chunk.len());
            h.update(&chunk[..n]);
            left -= n;
        }
        assert_eq!(h.finish()[..4], [0xcd, 0xc7, 0x6e, 0x5c]);
    }
}
