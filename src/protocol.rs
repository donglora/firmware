//! Wire protocol types and fixed-size little-endian serialization.
//!
//! Every integer is fixed-width LE. No varints, no zigzag.
//! See `docs/PROTOCOL.md` for the complete specification.

use heapless::Vec;

/// Maximum LoRa payload size in bytes.
pub const MAX_PAYLOAD: usize = 256;

/// RadioConfig wire size (fixed).
pub const RADIO_CONFIG_SIZE: usize = 13;

/// Sentinel value for `tx_power_dbm`: use the board's maximum TX power.
pub const TX_POWER_MAX: i8 = i8::MIN; // -128 on the wire

/// Sentinel value for `preamble_len`: use the firmware default (16 symbols).
pub const PREAMBLE_DEFAULT: u16 = 0;

/// LoRa signal bandwidth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(not(test), derive(defmt::Format))]
#[repr(u8)]
pub enum Bandwidth {
    Khz7 = 0,
    Khz10 = 1,
    Khz15 = 2,
    Khz20 = 3,
    Khz31 = 4,
    Khz41 = 5,
    Khz62 = 6,
    Khz125 = 7,
    Khz250 = 8,
    Khz500 = 9,
}

impl Bandwidth {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Khz7),
            1 => Some(Self::Khz10),
            2 => Some(Self::Khz15),
            3 => Some(Self::Khz20),
            4 => Some(Self::Khz31),
            5 => Some(Self::Khz41),
            6 => Some(Self::Khz62),
            7 => Some(Self::Khz125),
            8 => Some(Self::Khz250),
            9 => Some(Self::Khz500),
            _ => None,
        }
    }
}

/// Complete LoRa radio configuration.
///
/// Wire layout (13 bytes, all little-endian):
/// ```text
/// [freq_hz:4] [bw:1] [sf:1] [cr:1] [sync_word:2] [tx_power_dbm:1] [preamble_len:2] [cad:1]
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(not(test), derive(defmt::Format))]
pub struct RadioConfig {
    /// Frequency in Hz (150_000_000 - 960_000_000 for SX1262).
    pub freq_hz: u32,
    pub bw: Bandwidth,
    /// Spreading factor (5-12).
    pub sf: u8,
    /// Coding rate denominator (5-8). E.g. 5 = CR 4/5, 8 = CR 4/8.
    pub cr: u8,
    pub sync_word: u16,
    /// Transmit power in dBm. Set to [`TX_POWER_MAX`] (-128) for the
    /// board's maximum.
    pub tx_power_dbm: i8,
    /// Preamble length in symbols. Set to [`PREAMBLE_DEFAULT`] (0) for
    /// the firmware default (16 symbols). Valid range: 6–65535.
    pub preamble_len: u16,
    /// Channel Activity Detection (listen-before-talk) before TX.
    /// 0 = disabled, non-zero = enabled. Default: 1 (enabled).
    pub cad: u8,
}

impl RadioConfig {
    /// Validate fields against hardware limits.
    pub fn validate(&self, power_range: (i8, i8)) -> Result<(), &'static str> {
        if !(150_000_000..=960_000_000).contains(&self.freq_hz) {
            return Err("frequency out of range (150-960 MHz)");
        }
        if !(5..=12).contains(&self.sf) {
            return Err("spreading factor out of range (5-12)");
        }
        if !(5..=8).contains(&self.cr) {
            return Err("coding rate out of range (5-8)");
        }
        if self.tx_power_dbm != TX_POWER_MAX
            && !(power_range.0..=power_range.1).contains(&self.tx_power_dbm)
        {
            return Err("TX power out of range for this board");
        }
        if self.preamble_len != PREAMBLE_DEFAULT && self.preamble_len < 6 {
            return Err("preamble length too short (min 6)");
        }
        Ok(())
    }

    /// Resolve sentinel values to concrete defaults.
    pub fn resolve(mut self, power_range: (i8, i8)) -> Self {
        if self.tx_power_dbm == TX_POWER_MAX {
            self.tx_power_dbm = power_range.1;
        }
        if self.preamble_len == PREAMBLE_DEFAULT {
            self.preamble_len = 16;
        }
        self
    }

    /// Whether CAD (listen-before-talk) is enabled.
    pub fn cad_enabled(&self) -> bool {
        self.cad != 0
    }

    /// Serialize to fixed-size LE bytes. Returns number of bytes written (always 13).
    pub fn write_to(self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.freq_hz.to_le_bytes());
        buf[4] = self.bw as u8;
        buf[5] = self.sf;
        buf[6] = self.cr;
        buf[7..9].copy_from_slice(&self.sync_word.to_le_bytes());
        buf[9] = self.tx_power_dbm as u8;
        buf[10..12].copy_from_slice(&self.preamble_len.to_le_bytes());
        buf[12] = self.cad;
        RADIO_CONFIG_SIZE
    }

    /// Deserialize from fixed-size LE bytes.
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < RADIO_CONFIG_SIZE {
            return None;
        }
        Some(Self {
            freq_hz: u32::from_le_bytes(buf[0..4].try_into().ok()?),
            bw: Bandwidth::from_u8(buf[4])?,
            sf: buf[5],
            cr: buf[6],
            sync_word: u16::from_le_bytes(buf[7..9].try_into().ok()?),
            tx_power_dbm: buf[9] as i8,
            preamble_len: u16::from_le_bytes(buf[10..12].try_into().ok()?),
            cad: buf[12],
        })
    }
}

/// Host → firmware commands.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, PartialEq)]
pub enum Command {
    Ping,
    GetConfig,
    SetConfig(RadioConfig),
    StartRx,
    StopRx,
    Transmit {
        config: Option<RadioConfig>,
        payload: Vec<u8, MAX_PAYLOAD>,
    },
    DisplayOn,
    DisplayOff,
    GetMac,
}

impl Command {
    /// Deserialize from a COBS-decoded frame.
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        let tag = *buf.first()?;
        let rest = &buf[1..];
        match tag {
            0 => Some(Self::Ping),
            1 => Some(Self::GetConfig),
            2 => Some(Self::SetConfig(RadioConfig::from_bytes(rest)?)),
            3 => Some(Self::StartRx),
            4 => Some(Self::StopRx),
            5 => {
                // Transmit: has_config:u8 + [RadioConfig if 1] + len:u16 LE + payload
                if rest.is_empty() {
                    return None;
                }
                let (config, pos) = if rest[0] == 0 {
                    (None, 1)
                } else if rest[0] == 1 && rest.len() > RADIO_CONFIG_SIZE {
                    (
                        Some(RadioConfig::from_bytes(&rest[1..])?),
                        1 + RADIO_CONFIG_SIZE,
                    )
                } else {
                    return None;
                };
                if rest.len() < pos + 2 {
                    return None;
                }
                let len = u16::from_le_bytes(rest[pos..pos + 2].try_into().ok()?) as usize;
                let data_start = pos + 2;
                if rest.len() < data_start + len {
                    return None;
                }
                let mut payload = Vec::new();
                let _ = payload.extend_from_slice(&rest[data_start..data_start + len]);
                Some(Self::Transmit { config, payload })
            }
            6 => Some(Self::DisplayOn),
            7 => Some(Self::DisplayOff),
            8 => Some(Self::GetMac),
            _ => None,
        }
    }
}

/// Firmware → host responses.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, PartialEq)]
pub enum Response {
    Pong,
    Config(RadioConfig),
    RxPacket {
        rssi: i16,
        snr: i16,
        payload: Vec<u8, MAX_PAYLOAD>,
    },
    TxDone,
    Ok,
    Error(ErrorCode),
    MacAddress([u8; 6]),
}

impl Response {
    /// Serialize to fixed-size LE bytes. Returns number of bytes written.
    pub fn write_to(self, buf: &mut [u8]) -> usize {
        match self {
            Self::Pong => {
                buf[0] = 0;
                1
            }
            Self::Config(cfg) => {
                buf[0] = 1;
                1 + cfg.write_to(&mut buf[1..])
            }
            Self::RxPacket { rssi, snr, payload } => {
                buf[0] = 2;
                buf[1..3].copy_from_slice(&rssi.to_le_bytes());
                buf[3..5].copy_from_slice(&snr.to_le_bytes());
                buf[5..7].copy_from_slice(&(payload.len() as u16).to_le_bytes());
                buf[7..7 + payload.len()].copy_from_slice(&payload);
                7 + payload.len()
            }
            Self::TxDone => {
                buf[0] = 3;
                1
            }
            Self::Ok => {
                buf[0] = 4;
                1
            }
            Self::Error(code) => {
                buf[0] = 5;
                buf[1] = code as u8;
                2
            }
            Self::MacAddress(mac) => {
                buf[0] = 6;
                buf[1..7].copy_from_slice(&mac);
                7
            }
        }
    }
}

/// Error codes reported to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(not(test), derive(defmt::Format))]
#[repr(u8)]
pub enum ErrorCode {
    InvalidConfig = 0,
    RadioBusy = 1,
    TxTimeout = 2,
    // 3 reserved (was CrcError — SX1262 silently drops bad-CRC packets)
    NotConfigured = 4,
    NoDisplay = 5,
}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> RadioConfig {
        RadioConfig {
            freq_hz: 915_000_000,
            bw: Bandwidth::Khz125,
            sf: 7,
            cr: 5,
            sync_word: 0x3444,
            tx_power_dbm: 22,
            preamble_len: 16,
            cad: 1,
        }
    }

    // ── RadioConfig roundtrip ───────────────────────────────────────

    #[test]
    fn radio_config_roundtrip() {
        let cfg = make_config();
        let mut buf = [0u8; RADIO_CONFIG_SIZE];
        let n = cfg.write_to(&mut buf);
        assert_eq!(n, RADIO_CONFIG_SIZE);
        assert_eq!(RadioConfig::from_bytes(&buf), Some(cfg));
    }

    #[test]
    fn radio_config_roundtrip_all_bandwidths() {
        for bw_val in 0u8..=9 {
            let bw = Bandwidth::from_u8(bw_val).unwrap();
            let cfg = RadioConfig {
                freq_hz: 433_000_000,
                bw,
                sf: 12,
                cr: 8,
                sync_word: 0x1234,
                tx_power_dbm: -9,
                preamble_len: 16,
                cad: 1,
            };
            let mut buf = [0u8; RADIO_CONFIG_SIZE];
            cfg.write_to(&mut buf);
            assert_eq!(RadioConfig::from_bytes(&buf), Some(cfg));
        }
    }

    #[test]
    fn radio_config_roundtrip_negative_power() {
        let cfg = RadioConfig {
            tx_power_dbm: TX_POWER_MAX,
            ..make_config()
        };
        let mut buf = [0u8; RADIO_CONFIG_SIZE];
        cfg.write_to(&mut buf);
        assert_eq!(RadioConfig::from_bytes(&buf), Some(cfg));
    }

    #[test]
    fn radio_config_from_short_buffer() {
        let buf = [0u8; RADIO_CONFIG_SIZE - 1];
        assert!(RadioConfig::from_bytes(&buf).is_none());
    }

    #[test]
    fn radio_config_from_empty_buffer() {
        assert!(RadioConfig::from_bytes(&[]).is_none());
    }

    #[test]
    fn radio_config_invalid_bandwidth() {
        let mut buf = [0u8; RADIO_CONFIG_SIZE];
        make_config().write_to(&mut buf);
        buf[4] = 255; // invalid bandwidth
        assert!(RadioConfig::from_bytes(&buf).is_none());
    }

    // ── RadioConfig::validate ───────────────────────────────────────

    #[test]
    fn validate_freq_boundaries() {
        let power_range = (-9, 22);
        let base = make_config();

        let mut cfg = RadioConfig {
            freq_hz: 150_000_000,
            ..base
        };
        assert!(cfg.validate(power_range).is_ok());

        cfg.freq_hz = 960_000_000;
        assert!(cfg.validate(power_range).is_ok());

        cfg.freq_hz = 149_999_999;
        assert!(cfg.validate(power_range).is_err());

        cfg.freq_hz = 960_000_001;
        assert!(cfg.validate(power_range).is_err());
    }

    #[test]
    fn validate_sf_boundaries() {
        let power_range = (-9, 22);
        let base = make_config();

        for sf in 5..=12 {
            assert!(RadioConfig { sf, ..base }.validate(power_range).is_ok());
        }
        assert!(RadioConfig { sf: 4, ..base }.validate(power_range).is_err());
        assert!(RadioConfig { sf: 13, ..base }
            .validate(power_range)
            .is_err());
    }

    #[test]
    fn validate_cr_boundaries() {
        let power_range = (-9, 22);
        let base = make_config();

        for cr in 5..=8 {
            assert!(RadioConfig { cr, ..base }.validate(power_range).is_ok());
        }
        assert!(RadioConfig { cr: 4, ..base }.validate(power_range).is_err());
        assert!(RadioConfig { cr: 9, ..base }.validate(power_range).is_err());
    }

    #[test]
    fn validate_tx_power_max_sentinel() {
        let cfg = RadioConfig {
            tx_power_dbm: TX_POWER_MAX,
            ..make_config()
        };
        assert!(cfg.validate((-9, 22)).is_ok());
    }

    #[test]
    fn validate_tx_power_out_of_range() {
        let cfg = RadioConfig {
            tx_power_dbm: 23,
            ..make_config()
        };
        assert!(cfg.validate((-9, 22)).is_err());

        let cfg = RadioConfig {
            tx_power_dbm: -10,
            ..make_config()
        };
        assert!(cfg.validate((-9, 22)).is_err());
    }

    #[test]
    fn validate_preamble_default_sentinel() {
        let cfg = RadioConfig {
            preamble_len: PREAMBLE_DEFAULT,
            ..make_config()
        };
        assert!(cfg.validate((-9, 22)).is_ok());
    }

    #[test]
    fn validate_preamble_boundaries() {
        let power_range = (-9, 22);
        let base = make_config();

        assert!(RadioConfig {
            preamble_len: 6,
            ..base
        }
        .validate(power_range)
        .is_ok());
        assert!(RadioConfig {
            preamble_len: 5,
            ..base
        }
        .validate(power_range)
        .is_err());
    }

    // ── RadioConfig::resolve ──────────────────────────────────────

    #[test]
    fn resolve_power_max_sentinel() {
        let cfg = RadioConfig {
            tx_power_dbm: TX_POWER_MAX,
            ..make_config()
        };
        assert_eq!(cfg.resolve((-9, 22)).tx_power_dbm, 22);
    }

    #[test]
    fn resolve_power_explicit_unchanged() {
        let cfg = RadioConfig {
            tx_power_dbm: 10,
            ..make_config()
        };
        assert_eq!(cfg.resolve((-9, 22)).tx_power_dbm, 10);
    }

    #[test]
    fn resolve_preamble_default() {
        let cfg = RadioConfig {
            preamble_len: PREAMBLE_DEFAULT,
            ..make_config()
        };
        assert_eq!(cfg.resolve((-9, 22)).preamble_len, 16);
    }

    #[test]
    fn resolve_preamble_explicit_unchanged() {
        let cfg = RadioConfig {
            preamble_len: 32,
            ..make_config()
        };
        assert_eq!(cfg.resolve((-9, 22)).preamble_len, 32);
    }

    // ── Command::from_bytes ─────────────────────────────────────────

    #[test]
    fn command_ping() {
        assert_eq!(Command::from_bytes(&[0]), Some(Command::Ping));
    }

    #[test]
    fn command_get_config() {
        assert_eq!(Command::from_bytes(&[1]), Some(Command::GetConfig));
    }

    #[test]
    fn command_set_config() {
        let cfg = make_config();
        let mut buf = [0u8; 1 + RADIO_CONFIG_SIZE];
        buf[0] = 2;
        cfg.write_to(&mut buf[1..]);
        assert_eq!(Command::from_bytes(&buf), Some(Command::SetConfig(cfg)));
    }

    #[test]
    fn command_start_stop_rx() {
        assert_eq!(Command::from_bytes(&[3]), Some(Command::StartRx));
        assert_eq!(Command::from_bytes(&[4]), Some(Command::StopRx));
    }

    #[test]
    fn command_transmit_no_config() {
        let payload = b"hello";
        let mut buf = [0u8; 64];
        buf[0] = 5; // tag
        buf[1] = 0; // has_config = false
        buf[2..4].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        buf[4..9].copy_from_slice(payload);

        match Command::from_bytes(&buf[..9]).unwrap() {
            Command::Transmit { config, payload: p } => {
                assert!(config.is_none());
                assert_eq!(p.as_slice(), b"hello");
            }
            _ => panic!("expected Transmit"),
        }
    }

    #[test]
    fn command_transmit_with_config() {
        let cfg = make_config();
        let mut buf = [0u8; 64];
        buf[0] = 5; // tag
        buf[1] = 1; // has_config = true
        cfg.write_to(&mut buf[2..]);
        let payload = b"test";
        let pos = 2 + RADIO_CONFIG_SIZE;
        buf[pos..pos + 2].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        buf[pos + 2..pos + 6].copy_from_slice(payload);

        match Command::from_bytes(&buf[..pos + 6]).unwrap() {
            Command::Transmit { config, payload: p } => {
                assert_eq!(config, Some(cfg));
                assert_eq!(p.as_slice(), b"test");
            }
            _ => panic!("expected Transmit"),
        }
    }

    #[test]
    fn command_transmit_empty_payload() {
        let mut buf = [0u8; 4];
        buf[0] = 5; // tag
        buf[1] = 0; // has_config = false
        buf[2..4].copy_from_slice(&0u16.to_le_bytes());

        match Command::from_bytes(&buf).unwrap() {
            Command::Transmit { config, payload } => {
                assert!(config.is_none());
                assert!(payload.is_empty());
            }
            _ => panic!("expected Transmit"),
        }
    }

    #[test]
    fn command_transmit_truncated() {
        // Tag only — missing has_config byte
        assert!(Command::from_bytes(&[5]).is_none());

        // has_config=1 but no config bytes
        assert!(Command::from_bytes(&[5, 1]).is_none());

        // has_config=0 but no length bytes
        assert!(Command::from_bytes(&[5, 0]).is_none());

        // has_config=0, length says 5 but only 2 bytes of payload
        let mut buf = [0u8; 6];
        buf[0] = 5;
        buf[1] = 0;
        buf[2..4].copy_from_slice(&5u16.to_le_bytes());
        buf[4] = 0xAA;
        buf[5] = 0xBB;
        assert!(Command::from_bytes(&buf).is_none());
    }

    #[test]
    fn command_display_and_mac() {
        assert_eq!(Command::from_bytes(&[6]), Some(Command::DisplayOn));
        assert_eq!(Command::from_bytes(&[7]), Some(Command::DisplayOff));
        assert_eq!(Command::from_bytes(&[8]), Some(Command::GetMac));
    }

    #[test]
    fn command_invalid_tag() {
        assert!(Command::from_bytes(&[9]).is_none());
        assert!(Command::from_bytes(&[255]).is_none());
    }

    #[test]
    fn command_empty_buffer() {
        assert!(Command::from_bytes(&[]).is_none());
    }

    // ── Response::write_to ──────────────────────────────────────────

    #[test]
    fn response_pong() {
        let mut buf = [0u8; 1];
        assert_eq!(Response::Pong.write_to(&mut buf), 1);
        assert_eq!(buf[0], 0);
    }

    #[test]
    fn response_config() {
        let cfg = make_config();
        let mut buf = [0u8; 1 + RADIO_CONFIG_SIZE];
        let n = Response::Config(cfg).write_to(&mut buf);
        assert_eq!(n, 1 + RADIO_CONFIG_SIZE);
        assert_eq!(buf[0], 1);
        assert_eq!(RadioConfig::from_bytes(&buf[1..]), Some(cfg));
    }

    #[test]
    fn response_rx_packet() {
        let mut payload = Vec::new();
        let _ = payload.extend_from_slice(b"data");
        let mut buf = [0u8; 64];
        let n = Response::RxPacket {
            rssi: -80,
            snr: 10,
            payload,
        }
        .write_to(&mut buf);
        assert_eq!(n, 7 + 4); // tag(1) + rssi(2) + snr(2) + len(2) + "data"(4)
        assert_eq!(buf[0], 2);
        assert_eq!(i16::from_le_bytes([buf[1], buf[2]]), -80);
        assert_eq!(i16::from_le_bytes([buf[3], buf[4]]), 10);
        assert_eq!(u16::from_le_bytes([buf[5], buf[6]]), 4);
        assert_eq!(&buf[7..11], b"data");
    }

    #[test]
    fn response_tx_done() {
        let mut buf = [0u8; 1];
        assert_eq!(Response::TxDone.write_to(&mut buf), 1);
        assert_eq!(buf[0], 3);
    }

    #[test]
    fn response_ok() {
        let mut buf = [0u8; 1];
        assert_eq!(Response::Ok.write_to(&mut buf), 1);
        assert_eq!(buf[0], 4);
    }

    #[test]
    fn response_error_codes() {
        let mut buf = [0u8; 2];
        for (code, val) in [
            (ErrorCode::InvalidConfig, 0),
            (ErrorCode::RadioBusy, 1),
            (ErrorCode::TxTimeout, 2),
            (ErrorCode::NotConfigured, 4),
            (ErrorCode::NoDisplay, 5),
        ] {
            let n = Response::Error(code).write_to(&mut buf);
            assert_eq!(n, 2);
            assert_eq!(buf[0], 5);
            assert_eq!(buf[1], val);
        }
    }

    #[test]
    fn response_mac_address() {
        let mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let mut buf = [0u8; 7];
        let n = Response::MacAddress(mac).write_to(&mut buf);
        assert_eq!(n, 7);
        assert_eq!(buf[0], 6);
        assert_eq!(&buf[1..7], &mac);
    }
}
