//! Shared types and initialization helpers for ESP32-S3 boards.
//!
//! All ESP32-S3 boards in this project share the same SPI/DMA, USB OTG,
//! and I2C peripheral wiring. This module provides the common type aliases,
//! peripheral bundle structs, and init functions. Each board file re-exports
//! these types and calls the helpers from its own `Board::into_parts()`.

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Delay;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use static_cell::StaticCell;

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx1262, Sx126x};

// ── Concrete peripheral types ────────────────────────────────────────

type SpiBus = SpiDmaBus<'static, esp_hal::Async>;
type Nss = Output<'static>;
type Iv = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioSpiDevice = SpiDevice<'static, NoopRawMutex, SpiBus, Nss>;
pub type RadioDriver = Sx126x<RadioSpiDevice, Iv, Sx1262>;

#[cfg(not(feature = "heltec_v3"))]
pub type UsbDriver = esp_hal::otg_fs::asynch::Driver<'static>;

#[cfg(feature = "heltec_v3")]
pub type UartDriver = esp_hal::uart::Uart<'static, esp_hal::Async>;

pub type DisplayI2c = I2c<'static, esp_hal::Async>;

// ── Peripheral bundles ───────────────────────────────────────────────

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

// ── Shared initialization helpers ───────────────────────────────────

/// Start the ESP-RTOS scheduler and Embassy time driver.
pub fn start_timer(timg0: esp_hal::peripherals::TIMG0<'static>) {
    let timg0 = TimerGroup::new(timg0);
    esp_rtos::start(timg0.timer0);
}

/// Initialize SPI + DMA + SX1262 radio.
///
/// All current ESP32-S3 boards use the same SPI/radio pin mapping:
/// SCK=GPIO9, MOSI=GPIO10, MISO=GPIO11, NSS=GPIO8,
/// Reset=GPIO12, DIO1=GPIO14, BUSY=GPIO13.
#[allow(clippy::too_many_arguments)]
pub fn init_radio(
    spi2: esp_hal::peripherals::SPI2<'static>,
    dma_ch0: esp_hal::peripherals::DMA_CH0<'static>,
    sck: esp_hal::peripherals::GPIO9<'static>,
    mosi: esp_hal::peripherals::GPIO10<'static>,
    miso: esp_hal::peripherals::GPIO11<'static>,
    nss_pin: esp_hal::peripherals::GPIO8<'static>,
    reset_pin: esp_hal::peripherals::GPIO12<'static>,
    dio1_pin: esp_hal::peripherals::GPIO14<'static>,
    busy_pin: esp_hal::peripherals::GPIO13<'static>,
) -> RadioParts {
    let spi = Spi::new(
        spi2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(SpiMode::_0),
    )
    .expect("SPI init")
    .with_sck(sck)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_dma(dma_ch0);

    let (rx_buf, rx_desc, tx_buf, tx_desc) = esp_hal::dma_buffers!(256);
    let dma_rx = DmaRxBuf::new(rx_desc, rx_buf).expect("DMA RX buf");
    let dma_tx = DmaTxBuf::new(tx_desc, tx_buf).expect("DMA TX buf");

    let spi = spi.with_buffers(dma_rx, dma_tx).into_async();

    static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus>> =
        StaticCell::new();
    let spi_bus = SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi));

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

#[cfg(not(feature = "heltec_v3"))]
/// Initialize USB OTG CDC-ACM driver.
pub fn init_usb(
    usb0: esp_hal::peripherals::USB0<'static>,
    dp: esp_hal::peripherals::GPIO20<'static>,
    dm: esp_hal::peripherals::GPIO19<'static>,
) -> UsbParts {
    let usb_inst = esp_hal::otg_fs::Usb::new(usb0, dp, dm);
    static EP_OUT_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let ep_out_buf = EP_OUT_BUF.init([0u8; 1024]);
    UsbParts {
        driver: esp_hal::otg_fs::asynch::Driver::new(
            usb_inst,
            ep_out_buf,
            esp_hal::otg_fs::asynch::Config::default(),
        ),
    }
}

#[cfg(feature = "heltec_v3")]
/// Initialize UART0 for boards with USB-UART bridge chips (e.g. CP2102).
pub fn init_uart(
    uart0: esp_hal::peripherals::UART0<'static>,
    tx: esp_hal::peripherals::GPIO43<'static>,
    rx: esp_hal::peripherals::GPIO44<'static>,
) -> UartParts {
    use esp_hal::uart::{Config as UartConfig, Uart};

    let uart = Uart::new(uart0, UartConfig::default())
        .expect("UART init")
        .with_tx(tx)
        .with_rx(rx)
        .into_async();

    UartParts { driver: uart }
}

/// Initialize I2C bus for display.
pub fn init_display_i2c(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO17<'static>,
    scl: esp_hal::peripherals::GPIO18<'static>,
) -> DisplayI2c {
    I2c::new(i2c0, I2cConfig::default())
        .expect("I2C init")
        .with_sda(sda)
        .with_scl(scl)
        .into_async()
}
