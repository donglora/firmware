//! ESP32-S3 MCU initialization primitives.
//!
//! Provides low-level bus and peripheral init for ESP32-S3 boards.
//! Board files call these with specific pins and construct higher-level
//! drivers (Sx126x, SSD1306, etc.) themselves.

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use static_cell::StaticCell;

// ── MCU-level types ─────────────────────────────────────────────────

pub type SpiBus = SpiDmaBus<'static, esp_hal::Async>;
pub type I2cBus = I2c<'static, esp_hal::Async>;
#[allow(dead_code)] // Used by USB boards, not UART boards
pub type UsbOtgDriver = esp_hal::otg_fs::asynch::Driver<'static>;
#[allow(dead_code)] // Used by UART boards, not USB boards
pub type UartDriver = esp_hal::uart::Uart<'static, esp_hal::Async>;

// ── Timer ───────────────────────────────────────────────────────────

/// Start the ESP-RTOS scheduler and Embassy time driver.
pub fn start_timer(timg0: esp_hal::peripherals::TIMG0<'static>) {
    let timg0 = TimerGroup::new(timg0);
    esp_rtos::start(timg0.timer0);
}

// ── SPI bus ─────────────────────────────────────────────────────────

/// Initialize SPI2 with DMA and wrap in a shared bus (StaticCell + Mutex).
///
/// Returns a reference to the shared SPI bus suitable for `SpiDevice::new()`.
pub fn init_spi(
    spi2: esp_hal::peripherals::SPI2<'static>,
    dma_ch0: esp_hal::peripherals::DMA_CH0<'static>,
    sck: esp_hal::peripherals::GPIO9<'static>,
    mosi: esp_hal::peripherals::GPIO10<'static>,
    miso: esp_hal::peripherals::GPIO11<'static>,
) -> &'static embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus> {
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
    SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi))
}

// ── I2C bus ─────────────────────────────────────────────────────────

/// Initialize I2C0 for peripherals (display, sensors, etc.).
pub fn init_i2c(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO17<'static>,
    scl: esp_hal::peripherals::GPIO18<'static>,
) -> I2cBus {
    I2c::new(i2c0, I2cConfig::default())
        .expect("I2C init")
        .with_sda(sda)
        .with_scl(scl)
        .into_async()
}

// ── USB OTG ─────────────────────────────────────────────────────────

/// Initialize USB OTG FS driver for CDC-ACM.
#[allow(dead_code)] // Used by USB boards, not UART boards
pub fn init_usb(
    usb0: esp_hal::peripherals::USB0<'static>,
    dp: esp_hal::peripherals::GPIO20<'static>,
    dm: esp_hal::peripherals::GPIO19<'static>,
) -> UsbOtgDriver {
    let usb_inst = esp_hal::otg_fs::Usb::new(usb0, dp, dm);
    static EP_OUT_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let ep_out_buf = EP_OUT_BUF.init([0u8; 1024]);
    esp_hal::otg_fs::asynch::Driver::new(
        usb_inst,
        ep_out_buf,
        esp_hal::otg_fs::asynch::Config::default(),
    )
}

// ── UART ────────────────────────────────────────────────────────────

/// Initialize UART0 for boards with USB-UART bridge chips (e.g. CP2102).
#[allow(dead_code)] // Used by UART boards, not USB boards
pub fn init_uart(
    uart0: esp_hal::peripherals::UART0<'static>,
    tx: esp_hal::peripherals::GPIO43<'static>,
    rx: esp_hal::peripherals::GPIO44<'static>,
) -> UartDriver {
    use esp_hal::uart::{Config as UartConfig, Uart};

    Uart::new(uart0, UartConfig::default())
        .expect("UART init")
        .with_tx(tx)
        .with_rx(rx)
        .into_async()
}

// ── MAC address ─────────────────────────────────────────────────────

/// Read the factory-programmed MAC address from eFuse.
pub fn mac_address() -> [u8; 6] {
    esp_hal::efuse::Efuse::mac_address()
}
