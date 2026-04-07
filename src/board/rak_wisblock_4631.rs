use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::twim;
use embassy_nrf::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Delay;

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx1262, Sx126x};

use super::traits::LoRaBoard;
use crate::hal::nrf52840 as mcu;

// ── Concrete peripheral types ───────────────────────────────────────

type Nss = Output<'static>;
type Iv = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioSpiDevice = SpiDevice<'static, NoopRawMutex, mcu::SpiBus, Nss>;
pub type RadioDriver = Sx126x<RadioSpiDevice, Iv, Sx1262>;
pub type UsbDriver = mcu::UsbNrfDriver;
pub type DisplayI2c = mcu::I2cBus;

// ── Peripheral bundles ──────────────────────────────────────────────

pub struct RadioParts {
    pub driver: RadioDriver,
    pub delay: Delay,
}

pub struct UsbParts {
    pub driver: UsbDriver,
}

pub struct DisplayParts {
    pub i2c: DisplayI2c,
    pub mac: [u8; 6],
}

// ── Board init ──────────────────────────────────────────────────────

pub struct Board {
    p: embassy_nrf::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "RAK WisBlock 4631";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22);

    fn init() -> Self {
        let p = embassy_nrf::init(Default::default());
        Self { p }
    }

    fn mac_address() -> [u8; 6] {
        mcu::mac_address()
    }
}

impl Board {
    pub fn into_parts(self) -> (RadioParts, UsbParts, Option<DisplayParts>) {
        let p = self.p;

        // ── SPI bus for SX1262 ──────────────────────────────────
        let mut spi_cfg = spim::Config::default();
        spi_cfg.frequency = spim::Frequency::M1;
        let spi = Spim::new(p.SPI3, mcu::Irqs, p.P1_11, p.P1_13, p.P1_12, spi_cfg);
        let spi_bus = mcu::share_spi_bus(spi);

        let nss = Output::new(p.P1_10, Level::High, OutputDrive::Standard);
        let spi_device = SpiDevice::new(spi_bus, nss);

        // ── SX1262 control pins ─────────────────────────────────
        let reset = Output::new(p.P1_06, Level::High, OutputDrive::Standard);
        let dio1 = Input::new(p.P1_15, Pull::Down);
        let busy = Input::new(p.P1_14, Pull::Down);

        let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None)
            .expect("SX1262 interface init");

        let sx_config = sx126x::Config {
            chip: Sx1262,
            tcxo_ctrl: Some(sx126x::TcxoCtrlVoltage::Ctrl1V8),
            use_dcdc: true,
            rx_boost: false,
        };

        let radio = RadioParts {
            driver: Sx126x::new(spi_device, iv, sx_config),
            delay: Delay,
        };

        // ── USB ─────────────────────────────────────────────────
        let vbus = mcu::alloc_vbus_detect(true, false);
        let usb = UsbParts {
            driver: Driver::new(p.USBD, mcu::Irqs, vbus),
        };

        // ── Display (optional RAK1921 SSD1306 OLED on I2C) ─────
        let twim_buf = mcu::alloc_i2c_buffer();
        let i2c = embassy_nrf::twim::Twim::new(
            p.TWISPI0, mcu::Irqs, p.P0_13, p.P0_14, twim::Config::default(), twim_buf,
        );
        let mac = Self::mac_address();
        let display = Some(DisplayParts { i2c, mac });

        (radio, usb, display)
    }
}
