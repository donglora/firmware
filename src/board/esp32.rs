//! Shared types and peripheral wiring for classic-ESP32 SX1276 boards.
//!
//! Counterpart to `board::esp32s3` for the SX126x line. Builds an
//! `Sx127x<Sx1276>` driver from an already-initialized SPI bus plus the
//! board's reset and DIO0 pins. SX127x has no `BUSY` line — DIO0 carries
//! every IRQ (TX done, RX done, header valid, …) and lora-phy's
//! `GenericSx127xInterfaceVariant` waits on it directly.

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Delay;
use esp_hal::gpio::{Input, InputConfig, InputPin, Level, Output, OutputConfig, OutputPin, Pull};

use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{self, Sx127x};

use crate::hal::esp32 as mcu;

// ── Re-export MCU types ─────────────────────────────────────────────

pub use mcu::{I2cBus as DisplayI2c, SpiBus};

// ── Concrete peripheral types ───────────────────────────────────────

type Nss = Output<'static>;
type Iv = GenericSx127xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioSpiDevice = SpiDevice<'static, NoopRawMutex, SpiBus, Nss>;
pub type RadioDriver = Sx127x<RadioSpiDevice, Iv, sx127x::Sx1276>;

// ── Peripheral bundles ──────────────────────────────────────────────

pub struct RadioParts {
    pub driver: RadioDriver,
    pub delay: Delay,
}

pub struct DisplayParts {
    pub i2c: DisplayI2c,
}

// ── Display driver ──────────────────────────────────────────────────

pub type DisplayDriver = ssd1306::Ssd1306Async<
    ssd1306::prelude::I2CInterface<DisplayI2c>,
    ssd1306::size::DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsModeAsync<ssd1306::size::DisplaySize128x64>,
>;

/// Construct and initialize an SSD1306 display from raw I²C.
pub async fn create_display(i2c: DisplayI2c) -> Option<DisplayDriver> {
    use ssd1306::mode::DisplayConfigAsync;
    use ssd1306::prelude::{Brightness, DisplayRotation};
    use ssd1306::size::DisplaySize128x64;
    use ssd1306::{I2CDisplayInterface, Ssd1306Async};

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    embassy_time::Timer::after_millis(100).await;
    if display.init().await.is_err() {
        defmt::error!("SSD1306 display init failed");
        return None;
    }
    let _ = display.set_brightness(Brightness::BRIGHTEST).await;
    Some(display)
}

// ── Shared peripheral init helpers ──────────────────────────────────

/// Construct an SX1276 radio from an initialized SPI bus.
///
/// `dio0` is the primary IRQ (TX/RX done). The SX1276 module on the T-Beam
/// runs from a crystal oscillator, not a TCXO, and power comes from the
/// AXP192's PA_BOOST rail — `tx_boost = true` routes TX through the PA
/// output for +20 dBm.
pub fn init_radio(
    spi_bus: &'static embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus>,
    nss_pin: impl OutputPin + 'static,
    reset_pin: impl OutputPin + 'static,
    dio0_pin: impl InputPin + 'static,
) -> RadioParts {
    let nss = Output::new(nss_pin, Level::High, OutputConfig::default());
    let spi_device = SpiDevice::new(spi_bus, nss);

    let reset = Output::new(reset_pin, Level::High, OutputConfig::default());
    let dio0 = Input::new(dio0_pin, InputConfig::default().with_pull(Pull::Down));

    let iv =
        GenericSx127xInterfaceVariant::new(reset, dio0, None, None).expect("SX1276 interface init");

    let sx_config = sx127x::Config {
        chip: sx127x::Sx1276,
        tcxo_used: false,
        tx_boost: true,
        rx_boost: false,
    };

    RadioParts {
        driver: Sx127x::new(spi_device, iv, sx_config),
        delay: Delay,
    }
}
