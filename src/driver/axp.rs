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

/// LDO on/off control register. Bits:
///   0 = ALDO1, 1 = ALDO2, 2 = ALDO3, 3 = ALDO4, 4 = BLDO1, 5 = BLDO2.
const AXP2101_REG_LDO_ONOFF: u8 = 0x90;
/// ALDO1 voltage register (OLED rail on T-Beam Supreme).
const AXP2101_REG_ALDO1_V: u8 = 0x92;
/// ALDO3 voltage register (LoRa rail on T-Beam Supreme).
const AXP2101_REG_ALDO3_V: u8 = 0x94;

/// Voltage code for 3.3 V on ALDOx: 0.5 V + 28 × 100 mV.
const AXP2101_V_3V3: u8 = 28;

/// Mask enabling ALDO1 + ALDO3 in [`AXP2101_REG_LDO_ONOFF`].
const AXP2101_ALDO1_ALDO3: u8 = 0b0000_0101;

/// Bring up LoRa (ALDO3) and OLED (ALDO1) rails at 3.3 V.
///
/// Voltage registers are set before the enable bits to avoid glitching the
/// rails through the reset-default voltage. Other rails are left untouched
/// (read-modify-write on the enable register) so any board-specific OTP
/// defaults stay intact.
pub fn axp2101_init_lora_oled<I: I2c>(i2c: &mut I) -> Result<(), I::Error> {
    i2c.write(PMIC_ADDR, &[AXP2101_REG_ALDO3_V, AXP2101_V_3V3])?;
    i2c.write(PMIC_ADDR, &[AXP2101_REG_ALDO1_V, AXP2101_V_3V3])?;

    let mut onoff = [0u8; 1];
    i2c.write_read(PMIC_ADDR, &[AXP2101_REG_LDO_ONOFF], &mut onoff)?;
    i2c.write(
        PMIC_ADDR,
        &[AXP2101_REG_LDO_ONOFF, onoff[0] | AXP2101_ALDO1_ALDO3],
    )?;

    Ok(())
}
