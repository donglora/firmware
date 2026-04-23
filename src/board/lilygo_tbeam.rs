//! LilyGo T-Beam (classic) — ESP32 + SX1276 + SSD1306 OLED + AXP192 PMIC.
//!
//! First-generation T-Beam. ESP32 (dual-core Xtensa LX6, no native USB),
//! SX1276 module with XTAL (no TCXO), 128×64 SSD1306 OLED, AXP192 PMIC
//! gating the LoRa and OLED rails, CP2102 USB-UART bridge for host I/O.
//!
//! The AXP192 shares an I²C bus with the OLED (SDA=21, SCL=22). PMIC init
//! runs blocking on that bus first, then the same pins are re-initialized
//! in async mode for the display task.
//!
//! Pin map (verified from MeshCore variants/lilygo_tbeam_SX1276/):
//!   LoRa SPI:  SCK=5, MOSI=27, MISO=19, CS=18
//!   LoRa ctl:  RESET=23, DIO0=26 (primary IRQ — SX127x has no BUSY)
//!   OLED I²C:  SDA=21, SCL=22, SSD1306 @ 0x3C (shared with AXP192 @ 0x34)
//!   UART host: TX=1, RX=3 via CP2102 bridge (USB 10c4:ea60)
//!   LED:       GPIO4 (active HIGH — reuses MeshCore's P_LORA_TX_LED)
//!
//! GPS / battery / IMU / PCF8563 RTC are populated on the board but unused
//! by this firmware.

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};

use super::esp32;
use super::traits::{BoardParts, LoRaBoard};
use crate::driver::axp;
use crate::driver::simple_led::SimpleLed;
use crate::hal::esp32 as mcu;
use crate::protocol::RadioChipId;

pub use super::esp32::{create_display, DisplayDriver, DisplayParts, RadioDriver, RadioParts};

// ── Concrete peripheral types ───────────────────────────────────────

pub type UartDriver = esp_hal::uart::Uart<'static, esp_hal::Async>;
pub type LedDriver = SimpleLed<Output<'static>>;

// ── Peripheral bundles ──────────────────────────────────────────────

pub struct UartParts {
    pub driver: UartDriver,
}

// ── Board init ──────────────────────────────────────────────────────

pub struct Board {
    p: esp_hal::peripherals::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "LilyGo T-Beam";
    // SX1276 PA_BOOST range per the datasheet: +2 dBm min, +20 dBm max.
    const TX_POWER_RANGE: (i8, i8) = (2, 20);

    type RadioParts = RadioParts;
    type CommParts = UartParts;
    type DisplayParts = DisplayParts;
    type DisplayDriver = DisplayDriver;
    type LedDriver = LedDriver;

    fn init() -> Self {
        let p = esp_hal::init(esp_hal::Config::default());
        Self { p }
    }

    fn mac_address() -> [u8; 6] {
        mcu::mac_address()
    }

    fn radio_chip_id() -> u16 {
        RadioChipId::Sx1276.as_u16()
    }

    fn freq_range_hz() -> (u32, u32) {
        // SX1276 tuning window per datasheet (LF + HF bands combined).
        (137_000_000, 1_020_000_000)
    }

    fn supported_sf_bitmap() -> u16 {
        // SX127x supports SF6–SF12 (no SF5 like the SX126x).
        0x1FC0
    }

    fn into_parts(self) -> BoardParts<RadioParts, UartParts, DisplayParts, LedDriver> {
        let p = self.p;

        mcu::start_timer(p.TIMG0, p.SW_INTERRUPT);

        // AXP192 and SSD1306 share I²C0 (SDA=21, SCL=22). Build the bus
        // once as blocking, drive the PMIC init, then convert to async for
        // the display task — avoids re-consuming the I²C0 peripheral.
        // PMIC failure isn't fatal to boot: log and proceed so the rest of
        // the firmware can still emit diagnostics.
        let mut i2c_blocking = I2c::new(p.I2C0, I2cConfig::default())
            .expect("I2C0 init")
            .with_sda(p.GPIO21)
            .with_scl(p.GPIO22);
        if let Err(_e) = axp::axp192_init_lora_oled(&mut i2c_blocking) {
            defmt::error!("AXP192 init failed — LoRa rail may be dead");
        }
        // Let DCDC1 / LDO2 settle before SPI or display traffic.
        esp_hal::delay::Delay::new().delay_millis(20);

        // SPI2 for SX1276: SCK=5, MOSI=27, MISO=19
        let spi_bus = mcu::init_spi(p.SPI2, p.DMA_SPI2, p.GPIO5, p.GPIO27, p.GPIO19);

        // SX1276: CS=18, RESET=23, DIO0=26 (primary IRQ)
        let radio = esp32::init_radio(spi_bus, p.GPIO18, p.GPIO23, p.GPIO26);

        // UART0 via CP2102 bridge: TX=1, RX=3 (ESP32 default console)
        use esp_hal::uart::{Config as UartConfig, Uart};
        let host = UartParts {
            driver: Uart::new(p.UART0, UartConfig::default())
                .expect("UART init")
                .with_tx(p.GPIO1)
                .with_rx(p.GPIO3)
                .into_async(),
        };

        // Hand the (now async) I²C bus to the SSD1306 display task.
        let display = Some(DisplayParts {
            i2c: i2c_blocking.into_async(),
        });

        // TX activity LED on GPIO4 (active HIGH).
        let led_pin = Output::new(p.GPIO4, Level::Low, OutputConfig::default());
        let led = SimpleLed(led_pin);

        BoardParts {
            radio,
            host,
            display,
            led,
            mac: Self::mac_address(),
        }
    }
}
