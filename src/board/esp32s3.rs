//! Shared types and peripheral wiring for ESP32-S3 boards.
//!
//! Delegates MCU-level init to `hal::esp32s3` and defines peripheral-level
//! types (RadioDriver, Parts structs) used by ESP32-S3 board files.

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Delay;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx1262, Sx126x};

use crate::hal::esp32s3 as mcu;

// ── Re-export MCU types ─────────────────────────────────────────────

pub use mcu::{I2cBus as DisplayI2c, SpiBus, UartDriver, UsbOtgDriver as UsbDriver};

// ── Concrete peripheral types ───────────────────────────────────────

type Nss = Output<'static>;
type Iv = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioSpiDevice = SpiDevice<'static, NoopRawMutex, SpiBus, Nss>;
pub type RadioDriver = Sx126x<RadioSpiDevice, Iv, Sx1262>;

// ── Peripheral bundles ──────────────────────────────────────────────

pub struct RadioParts {
    pub driver: RadioDriver,
    pub delay: Delay,
}

#[cfg(not(feature = "heltec_v3"))]
pub struct UsbParts {
    pub driver: UsbDriver,
}

#[cfg(feature = "heltec_v3")]
pub struct UartParts {
    pub driver: UartDriver,
}

pub struct DisplayParts {
    pub i2c: DisplayI2c,
    pub mac: [u8; 6],
}

// ── Shared peripheral init helpers ──────────────────────────────────

/// Construct SX1262 radio from an initialized SPI bus.
pub fn init_radio(
    spi_bus: &'static embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus>,
    nss_pin: esp_hal::peripherals::GPIO8<'static>,
    reset_pin: esp_hal::peripherals::GPIO12<'static>,
    dio1_pin: esp_hal::peripherals::GPIO14<'static>,
    busy_pin: esp_hal::peripherals::GPIO13<'static>,
) -> RadioParts {
    let nss = Output::new(nss_pin, Level::High, OutputConfig::default());
    let spi_device = SpiDevice::new(spi_bus, nss);

    let reset = Output::new(reset_pin, Level::High, OutputConfig::default());
    let dio1 = Input::new(dio1_pin, InputConfig::default().with_pull(Pull::Down));
    let busy = Input::new(busy_pin, InputConfig::default().with_pull(Pull::Down));

    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None)
        .expect("SX1262 interface init");

    let sx_config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(sx126x::TcxoCtrlVoltage::Ctrl1V8),
        use_dcdc: true,
        rx_boost: false,
    };

    RadioParts {
        driver: Sx126x::new(spi_device, iv, sx_config),
        delay: Delay,
    }
}
