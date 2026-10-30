//! Decoding of `diagnostic_msgs/msg/DiagnosticArray` and the table of
//! statuses the diagnostics viewer builds from many partial arrays.

use super::cdr::{CdrError, CdrReader};
use std::collections::HashMap;
use std::time::{Duration, Instant};

pub const DIAGNOSTIC_ARRAY_TYPE: &str = "diagnostic_msgs/msg/DiagnosticArray";

/// A status not updated for this long is shown as stale, like the
/// diagnostic aggregator does.
pub const STALE_AFTER: Duration = Duration::from_secs(5);

pub fn is_diagnostics_type(msg_type: &str) -> bool {
    msg_type == DIAGNOSTIC_ARRAY_TYPE
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Ok,
    Warn,
    Error,
    Stale,
}

impl Level {
    fn from_byte(level: u8) -> Self {
        match level {
            0 => Level::Ok,
            1 => Level::Warn,
            3 => Level::Stale,
            // Unknown levels are treated as errors rather than hidden.
            _ => Level::Error,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Level::Ok => "OK",
            Level::Warn => "WARN",
            Level::Error => "ERROR",
            Level::Stale => "STALE",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DiagnosticStatus {
    pub level: Level,
    pub name: String,
    pub message: String,
    pub hardware_id: String,
    pub values: Vec<(String, String)>,
}

pub fn decode_message(payload: &[u8]) -> Result<Vec<DiagnosticStatus>, CdrError> {
    let mut reader = CdrReader::new(payload)?;
    reader.header()?;
    let count = reader.u32()?;
    let mut statuses = Vec::new();
    for _ in 0..count {
        let level = Level::from_byte(reader.u8()?);
        let name = reader.string()?;
        let message = reader.string()?;
        let hardware_id = reader.string()?;
        let value_count = reader.u32()?;
        let mut values = Vec::new();
        for _ in 0..value_count {
            values.push((reader.string()?, reader.string()?));
        }
        statuses.push(DiagnosticStatus {
            level,
            name,
            message,
            hardware_id,
            values,
        });
    }
    Ok(statuses)
}

#[derive(Debug, Clone)]
pub struct Entry {
    pub status: DiagnosticStatus,
    pub updated: Instant,
}

impl Entry {
    /// The reported level, or stale once updates stop.
    pub fn level(&self, now: Instant) -> Level {
        if now.duration_since(self.updated) > STALE_AFTER {
            Level::Stale
        } else {
            self.status.level
        }
    }
}

/// Latest status per name, merged from all arrays received.
#[derive(Debug, Default)]
pub struct DiagnosticsTable {
    entries: HashMap<String, Entry>,
}

impl DiagnosticsTable {
    pub fn update(&mut self, statuses: Vec<DiagnosticStatus>, now: Instant) {
        for status in statuses {
            self.entries.insert(
                status.name.clone(),
                Entry {
                    status,
                    updated: now,
                },
            );
        }
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Entries worst first, then by name; only problems if `problems_only`.
    pub fn sorted(&self, now: Instant, problems_only: bool) -> Vec<&Entry> {
        let mut entries: Vec<&Entry> = self
            .entries
            .values()
            .filter(|e| !problems_only || e.level(now) != Level::Ok)
            .collect();
        entries.sort_by(|a, b| {
            b.level(now)
                .cmp(&a.level(now))
                .then_with(|| a.status.name.cmp(&b.status.name))
        });
        entries
    }

    /// Number of statuses per level: OK, WARN, ERROR, STALE.
    pub fn counts(&self, now: Instant) -> [usize; 4] {
        let mut counts = [0; 4];
        for entry in self.entries.values() {
            counts[entry.level(now) as usize] += 1;
        }
        counts
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::cdr::CdrWriter;

    /// (level, name, message, key/value pairs) per status.
    type TestStatus<'a> = (u8, &'a str, &'a str, &'a [(&'a str, &'a str)]);

    fn array_msg(statuses: &[TestStatus]) -> Vec<u8> {
        let mut w = CdrWriter::new();
        w.header("");
        w.u32(statuses.len() as u32);
        for (level, name, message, values) in statuses {
            w.u8(*level);
            w.string(name);
            w.string(message);
            w.string("hw-1");
            w.u32(values.len() as u32);
            for (k, v) in *values {
                w.string(k);
                w.string(v);
            }
        }
        w.0
    }

    #[test]
    fn decodes_statuses_with_values() {
        let msg = array_msg(&[
            (0, "lidar: link", "Connected", &[("packets/s", "1200")]),
            (2, "camera: driver", "No frames", &[]),
        ]);
        let statuses = decode_message(&msg).unwrap();
        assert_eq!(statuses.len(), 2);
        assert_eq!(statuses[0].level, Level::Ok);
        assert_eq!(statuses[0].hardware_id, "hw-1");
        assert_eq!(
            statuses[0].values,
            vec![("packets/s".into(), "1200".into())]
        );
        assert_eq!(statuses[1].level, Level::Error);
        assert_eq!(statuses[1].message, "No frames");
    }

    #[test]
    fn merges_partial_arrays_sorts_worst_first_and_goes_stale() {
        let t0 = Instant::now();
        let mut table = DiagnosticsTable::default();
        table.update(
            decode_message(&array_msg(&[(0, "a", "", &[])])).unwrap(),
            t0,
        );
        table.update(
            decode_message(&array_msg(&[(1, "b", "", &[])])).unwrap(),
            t0,
        );
        table.update(
            decode_message(&array_msg(&[(2, "c", "", &[])])).unwrap(),
            t0,
        );

        let names = |now, only| -> Vec<String> {
            table
                .sorted(now, only)
                .iter()
                .map(|e| e.status.name.clone())
                .collect()
        };
        assert_eq!(names(t0, false), ["c", "b", "a"]);
        assert_eq!(names(t0, true), ["c", "b"]);
        assert_eq!(table.counts(t0), [1, 1, 1, 0]);

        let later = t0 + STALE_AFTER + Duration::from_secs(1);
        assert_eq!(table.counts(later), [0, 0, 0, 3]);
    }
}
