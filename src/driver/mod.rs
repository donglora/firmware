#[cfg(feature = "wio_tracker_l1")]
pub mod sh1106;

#[cfg(any(feature = "heltec_v3", feature = "heltec_v3_uart", feature = "heltec_v4"))]
pub mod simple_led;
