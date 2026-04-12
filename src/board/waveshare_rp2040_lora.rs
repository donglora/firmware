use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Delay;

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx1262, Sx126x};

use super::traits::{BoardParts, LoRaBoard};
use crate::hal::rp2040 as mcu;

// ── Concrete peripheral types ──────────────────────────────────────

type Nss = Output<'static>;
type Iv = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioSpiDevice = SpiDevice<'static, NoopRawMutex, mcu::SpiBus, Nss>;
pub type RadioDriver = Sx126x<RadioSpiDevice, Iv, Sx1262>;
pub type UsbDriver = mcu::UsbRpDriver;
pub type LedDriver = crate::driver::simple_led::SimpleLed<Output<'static>>;

// Display types — never instantiated (no OLED on this board), but the
// trait requires concrete associated types that satisfy the bounds.
pub type DisplayI2c =
    embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>;
pub type DisplayDriver = ssd1306::Ssd1306Async<
    ssd1306::prelude::I2CInterface<DisplayI2c>,
    ssd1306::size::DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsModeAsync<ssd1306::size::DisplaySize128x64>,
>;

// ── Peripheral bundles ─────────────────────────────────────────────

pub struct RadioParts {
    pub driver: RadioDriver,
    pub delay: Delay,
}

pub struct UsbParts {
    pub driver: UsbDriver,
}

pub struct DisplayParts {
    pub i2c: DisplayI2c,
}

// ── Display init ───────────────────────────────────────────────────

pub async fn create_display(_i2c: DisplayI2c) -> Option<DisplayDriver> {
    None
}

// ── Board init ─────────────────────────────────────────────────────

pub struct Board {
    p: embassy_rp::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "Waveshare RP2040-LoRa";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22);

    type RadioParts = RadioParts;
    type CommParts = UsbParts;
    type DisplayParts = DisplayParts;
    type DisplayDriver = DisplayDriver;
    type LedDriver = LedDriver;

    fn init() -> Self {
        let p = embassy_rp::init(Default::default());
        Self { p }
    }

    fn mac_address() -> [u8; 6] {
        mcu::mac_address()
    }

    fn into_parts(self) -> BoardParts<RadioParts, UsbParts, DisplayParts, LedDriver> {
        let p = self.p;

        // ── MAC address from flash unique ID ───────────────────
        let mut flash =
            embassy_rp::flash::Flash::<_, _, { 2 * 1024 * 1024 }>::new_blocking(p.FLASH);
        mcu::init_mac(&mut flash);

        // ── SPI1 bus for SX1262 ────────────────────────────────
        //
        // Pin mapping from Meshtastic & MeshCore firmware (confirmed working):
        //   SCK = GPIO14, MOSI = GPIO15, MISO = GPIO24, CS = GPIO13
        //
        // The Waveshare wiki (https://www.waveshare.com/wiki/RP2040-LoRa) lists
        // different pins (MOSI=11, MISO=8, CS=9) but those don't work in practice.
        // Both sets are valid SPI1 alternate functions on RP2040.
        let mut spi_cfg = spi::Config::default();
        spi_cfg.frequency = 1_000_000;
        let spi = spi::Spi::new(
            p.SPI1,
            p.PIN_14,
            p.PIN_15,
            p.PIN_24,
            p.DMA_CH0,
            p.DMA_CH1,
            mcu::Irqs,
            spi_cfg,
        );
        let spi_bus = mcu::share_spi_bus(spi);

        let nss = Output::new(p.PIN_13, Level::High);
        let spi_device = SpiDevice::new(spi_bus, nss);

        let reset = Output::new(p.PIN_23, Level::High);
        let dio1 = Input::new(p.PIN_16, Pull::Down);
        let busy = Input::new(p.PIN_18, Pull::Down);

        // PE4259 antenna switch: GPIO17 = !CTRL (rx_enable).
        // DIO2 is internally wired as CTRL (tx_enable), managed by the SX1262.
        let rx_enable = Output::new(p.PIN_17, Level::Low);
        let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, Some(rx_enable), None)
            .expect("SX1262 interface init");

        let sx_config = sx126x::Config {
            chip: Sx1262,
            tcxo_ctrl: None, // No TCXO on this board
            use_dcdc: true,
            rx_boost: false,
        };

        let radio = RadioParts {
            driver: Sx126x::new(spi_device, iv, sx_config),
            delay: Delay,
        };

        // ── USB ────────────────────────────────────────────────
        let host = UsbParts {
            driver: embassy_rp::usb::Driver::new(p.USB, mcu::Irqs),
        };

        // ── LED on GPIO25 ──────────────────────────────────────
        let led_pin = Output::new(p.PIN_25, Level::Low);
        let led = crate::driver::simple_led::SimpleLed(led_pin);

        BoardParts {
            radio,
            host,
            display: None,
            led,
            mac: Self::mac_address(),
        }
    }
}
