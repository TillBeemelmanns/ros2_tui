//! Minimal reader for serialized (CDR) ROS2 messages, enough for the plain
//! (final) structs the viewers decode.

use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum CdrError {
    Truncated,
    UnsupportedEncapsulation(u16),
}

impl fmt::Display for CdrError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CdrError::Truncated => write!(f, "message is truncated"),
            CdrError::UnsupportedEncapsulation(kind) => {
                write!(f, "unsupported CDR encapsulation 0x{:04x}", kind)
            }
        }
    }
}

pub struct CdrReader<'a> {
    body: &'a [u8],
    pos: usize,
    little_endian: bool,
}

impl<'a> CdrReader<'a> {
    pub fn new(buf: &'a [u8]) -> Result<Self, CdrError> {
        if buf.len() < 4 {
            return Err(CdrError::Truncated);
        }
        let kind = u16::from_be_bytes([buf[0], buf[1]]);
        // XCDR1 and plain XCDR2 share one layout for the decoded messages:
        // they have no 8-byte members, the only place the alignment differs.
        let little_endian = match kind {
            0x0000 | 0x0006 => false,
            0x0001 | 0x0007 => true,
            other => return Err(CdrError::UnsupportedEncapsulation(other)),
        };
        Ok(Self {
            body: &buf[4..],
            pos: 0,
            little_endian,
        })
    }

    fn align(&mut self, n: usize) {
        self.pos = (self.pos + n - 1) & !(n - 1);
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], CdrError> {
        let end = self.pos.checked_add(n).ok_or(CdrError::Truncated)?;
        let slice = self.body.get(self.pos..end).ok_or(CdrError::Truncated)?;
        self.pos = end;
        Ok(slice)
    }

    pub fn u8(&mut self) -> Result<u8, CdrError> {
        Ok(self.take(1)?[0])
    }

    pub fn bool(&mut self) -> Result<bool, CdrError> {
        Ok(self.u8()? != 0)
    }

    pub fn u32(&mut self) -> Result<u32, CdrError> {
        self.align(4);
        let b: [u8; 4] = self.take(4)?.try_into().unwrap();
        Ok(if self.little_endian {
            u32::from_le_bytes(b)
        } else {
            u32::from_be_bytes(b)
        })
    }

    pub fn i32(&mut self) -> Result<i32, CdrError> {
        Ok(self.u32()? as i32)
    }

    pub fn f32(&mut self) -> Result<f32, CdrError> {
        Ok(f32::from_bits(self.u32()?))
    }

    pub fn f32_sequence(&mut self) -> Result<Vec<f32>, CdrError> {
        let len = self.u32()? as usize;
        let little_endian = self.little_endian;
        let bytes = self.take(len.checked_mul(4).ok_or(CdrError::Truncated)?)?;
        Ok(bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|&b| {
                if little_endian {
                    f32::from_le_bytes(b)
                } else {
                    f32::from_be_bytes(b)
                }
            })
            .collect())
    }

    pub fn string(&mut self) -> Result<String, CdrError> {
        let len = self.u32()? as usize;
        let bytes = self.take(len)?;
        // The serialized length includes the trailing NUL.
        let bytes = bytes.strip_suffix(&[0]).unwrap_or(bytes);
        Ok(String::from_utf8_lossy(bytes).into_owned())
    }

    pub fn byte_sequence(&mut self) -> Result<&'a [u8], CdrError> {
        let len = self.u32()? as usize;
        self.take(len)
    }

    /// `std_msgs/Header`, returned as (stamp in seconds, frame_id).
    pub fn header(&mut self) -> Result<(f64, String), CdrError> {
        let sec = self.i32()?;
        let nanosec = self.u32()?;
        let frame_id = self.string()?;
        Ok((sec as f64 + nanosec as f64 * 1e-9, frame_id))
    }
}

/// Serializes messages the way rmw does (little endian XCDR1), for tests.
#[cfg(test)]
pub struct CdrWriter(pub Vec<u8>);

#[cfg(test)]
impl Default for CdrWriter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
impl CdrWriter {
    pub fn new() -> Self {
        CdrWriter(vec![0x00, 0x01, 0x00, 0x00])
    }
    fn align(&mut self, n: usize) {
        while !(self.0.len() - 4).is_multiple_of(n) {
            self.0.push(0);
        }
    }
    pub fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    pub fn u32(&mut self, v: u32) {
        self.align(4);
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    pub fn f32(&mut self, v: f32) {
        self.u32(v.to_bits());
    }
    pub fn f32_sequence(&mut self, values: &[f32]) {
        self.u32(values.len() as u32);
        for v in values {
            self.0.extend_from_slice(&v.to_le_bytes());
        }
    }
    pub fn string(&mut self, s: &str) {
        self.u32(s.len() as u32 + 1);
        self.0.extend_from_slice(s.as_bytes());
        self.0.push(0);
    }
    pub fn bytes(&mut self, b: &[u8]) {
        self.u32(b.len() as u32);
        self.0.extend_from_slice(b);
    }
    pub fn header(&mut self, frame_id: &str) {
        self.u32(12);
        self.u32(500_000_000);
        self.string(frame_id);
    }
}
