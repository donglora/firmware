//! ESP32-S3 MCU initialization primitives.
//!
//! Provides low-level bus init for ESP32-S3 boards.
//! Board files call these with specific pins and construct higher-level
//! drivers (Sx126x, SSD1306, USB, UART, etc.) themselves.

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::interconnect::{PeripheralInput, PeripheralOutput};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use static_cell::StaticCell;

// ── MCU-level types ─────────────────────────────────────────────────

pub type SpiBus = SpiDmaBus<'static, esp_hal::Async>;
pub type I2cBus = I2c<'static, esp_hal::Async>;

// ── Timer ───────────────────────────────────────────────────────────

/// Start the ESP-RTOS scheduler and Embassy time driver.
pub fn start_timer(
    timg0: esp_hal::peripherals::TIMG0<'static>,
    sw_int: esp_hal::peripherals::SW_INTERRUPT<'static>,
) {
    let timg0 = TimerGroup::new(timg0);
    let sw_ints = esp_hal::interrupt::software::SoftwareInterruptControl::new(sw_int);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);
}

// ── SPI bus ─────────────────────────────────────────────────────────

/// Initialize SPI2 with DMA and wrap in a shared bus (StaticCell + Mutex).
///
/// Generic over pin type so boards with different SPI pin assignments can
/// share this helper. Any GPIO that implements the esp-hal interconnect
/// traits (which is every GPIO on the ESP32-S3) can be passed directly.
pub fn init_spi(
    spi2: esp_hal::peripherals::SPI2<'static>,
    dma_ch0: esp_hal::peripherals::DMA_CH0<'static>,
    sck: impl PeripheralOutput<'static>,
    mosi: impl PeripheralOutput<'static>,
    miso: impl PeripheralInput<'static>,
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
///
/// Generic over pin type. I2C lines are bidirectional (open-drain), so both
/// `PeripheralInput` and `PeripheralOutput` bounds are required.
pub fn init_i2c(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: impl PeripheralInput<'static> + PeripheralOutput<'static>,
    scl: impl PeripheralInput<'static> + PeripheralOutput<'static>,
) -> I2cBus {
    I2c::new(i2c0, I2cConfig::default())
        .expect("I2C init")
        .with_sda(sda)
        .with_scl(scl)
        .into_async()
}

// ── MAC address ─────────────────────────────────────────────────────

/// Read the factory-programmed MAC address from eFuse.
pub fn mac_address() -> [u8; 6] {
    esp_hal::efuse::base_mac_address()
        .as_bytes()
        .try_into()
        .expect("MAC is 6 bytes")
}
