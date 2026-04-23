//! AXP PMIC init — turn on the rails that feed LoRa and the OLED.
//!
//! T-Beam–class boards gate LoRa, OLED (and GPS/sensors, which this firmware
//! does not use) through an X-Powers PMIC on I²C. Until the PMIC is told to
//! enable the right LDOs/DCDCs, the SX126x/SX127x chip sits unpowered and
//! `LoRa::new` fails. This module's only job is to flip those switches.
//!
//! Two PMICs are covered:
//!
//! * [`axp2101_init_lora_oled`] — LilyGo T-Beam S3 Supreme. ALDO3 = 3.3 V
//!   feeds the SX1262; ALDO1 = 3.3 V feeds the SH1106 OLED. Both live on a
//!   dedicated I²C1 bus (separate from the display).
//! * [`axp192_init_lora_oled`] — LilyGo T-Beam (classic). LDO2 = 3.3 V feeds
//!   the SX1276; DCDC1 = 3.3 V feeds the SSD1306 OLED. PMIC shares the
//!   display I²C bus.
//!
//! Everything unrelated to LoRa + OLED (GPS, sensors, SD, batt ADC) is left
//! alone. Out-of-scope for a transparent LoRa-over-USB bridge.
//!
//! Init is blocking on purpose. It runs once during `into_parts`, which is a
//! synchronous function called before task spawning. Using the blocking
//! `embedded_hal` trait keeps the PMIC wiring out of the async machinery and
//! lets the caller drop the I²C bus immediately after init (no bus handoff).

use embedded_hal::i2c::I2c;

/// Both chips respond at this 7-bit I²C address.
const PMIC_ADDR: u8 = 0x34;

// ── AXP2101 (T-Beam S3 Supreme) ──────────────────────────────────────

#[cfg(feature = "lilygo_tbeam_supreme")]
mod axp2101 {
    use super::{I2c, PMIC_ADDR};

    /// LDO on/off control register. Bits:
    ///   0 = ALDO1, 1 = ALDO2, 2 = ALDO3, 3 = ALDO4, 4 = BLDO1, 5 = BLDO2.
    const REG_LDO_ONOFF: u8 = 0x90;
    /// ALDO1 voltage register (OLED rail on T-Beam Supreme).
    const REG_ALDO1_V: u8 = 0x92;
    /// ALDO3 voltage register (LoRa rail on T-Beam Supreme).
    const REG_ALDO3_V: u8 = 0x94;

    /// Voltage code for 3.3 V on ALDOx: 0.5 V + 28 × 100 mV.
    const V_3V3: u8 = 28;

    /// Mask enabling ALDO1 + ALDO3 in [`REG_LDO_ONOFF`].
    const ALDO1_ALDO3: u8 = 0b0000_0101;

    /// Bring up LoRa (ALDO3) and OLED (ALDO1) rails at 3.3 V.
    ///
    /// Voltage registers are set before the enable bits to avoid glitching
    /// the rails through the reset-default voltage. Other rails are left
    /// untouched (read-modify-write on the enable register) so any
    /// board-specific OTP defaults stay intact.
    pub fn init_lora_oled<I: I2c>(i2c: &mut I) -> Result<(), I::Error> {
        i2c.write(PMIC_ADDR, &[REG_ALDO3_V, V_3V3])?;
        i2c.write(PMIC_ADDR, &[REG_ALDO1_V, V_3V3])?;

        let mut onoff = [0u8; 1];
        i2c.write_read(PMIC_ADDR, &[REG_LDO_ONOFF], &mut onoff)?;
        i2c.write(PMIC_ADDR, &[REG_LDO_ONOFF, onoff[0] | ALDO1_ALDO3])?;

        Ok(())
    }
}

#[cfg(feature = "lilygo_tbeam_supreme")]
pub use axp2101::init_lora_oled as axp2101_init_lora_oled;

// ── AXP192 (T-Beam classic) ──────────────────────────────────────────

#[cfg(feature = "lilygo_tbeam")]
mod axp192 {
    use super::{I2c, PMIC_ADDR};

    /// DCDC/LDO enable register. Bit layout:
    ///   0 = DCDC1, 1 = DCDC3, 2 = LDO2, 3 = LDO3, 4 = DCDC2, 6 = EXTEN.
    const REG_ONOFF: u8 = 0x12;
    /// DCDC1 output voltage register (OLED rail on T-Beam classic).
    /// 7-bit code, 25 mV step, 0.7 V offset → 3.3 V = 104 = `0x68`.
    const REG_DCDC1_V: u8 = 0x26;
    /// LDO2/LDO3 voltage register. Upper nibble = LDO2 (LoRa), lower nibble
    /// = LDO3 (GPS). 100 mV step, 1.8 V offset → 3.3 V = 15 = `0xF`.
    const REG_LDO23_V: u8 = 0x28;

    /// DCDC1 code for 3.3 V.
    const DCDC1_3V3: u8 = 0x68;
    /// LDO2 nibble for 3.3 V in the upper half of [`REG_LDO23_V`].
    const LDO2_3V3_UPPER: u8 = 0xF0;

    /// Mask enabling DCDC1 + LDO2 in [`REG_ONOFF`].
    const DCDC1_LDO2: u8 = 0b0000_0101;

    /// Bring up LoRa (LDO2) and OLED (DCDC1) rails at 3.3 V on the
    /// classic T-Beam's AXP192.
    ///
    /// GPS (LDO3) and EXTEN are not touched — this firmware has no GPS,
    /// and any board-default for LDO3 voltage is preserved by
    /// read-modify-write. Voltage is programmed before the enable bit
    /// flips so the rail doesn't glitch through its reset default.
    pub fn init_lora_oled<I: I2c>(i2c: &mut I) -> Result<(), I::Error> {
        let mut ldo23 = [0u8; 1];
        i2c.write_read(PMIC_ADDR, &[REG_LDO23_V], &mut ldo23)?;
        i2c.write(
            PMIC_ADDR,
            &[REG_LDO23_V, (ldo23[0] & 0x0F) | LDO2_3V3_UPPER],
        )?;

        i2c.write(PMIC_ADDR, &[REG_DCDC1_V, DCDC1_3V3])?;

        let mut onoff = [0u8; 1];
        i2c.write_read(PMIC_ADDR, &[REG_ONOFF], &mut onoff)?;
        i2c.write(PMIC_ADDR, &[REG_ONOFF, onoff[0] | DCDC1_LDO2])?;

        Ok(())
    }
}

#[cfg(feature = "lilygo_tbeam")]
pub use axp192::init_lora_oled as axp192_init_lora_oled;
