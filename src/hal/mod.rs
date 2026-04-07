//! MCU family initialization primitives.
//!
//! Each submodule provides low-level bus and peripheral init for one MCU family.
//! Board files call these with specific pins and construct higher-level drivers.

#[cfg(any(feature = "heltec_v3", feature = "heltec_v4"))]
pub mod esp32s3;

#[cfg(any(feature = "rak_wisblock_4631", feature = "wio_tracker_l1"))]
pub mod nrf52840;
