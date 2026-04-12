#[cfg(any(feature = "wio_tracker_l1", feature = "elecrow_thinknode_m2"))]
pub mod sh1106;

#[cfg(any(
    feature = "heltec_v3",
    feature = "heltec_v3_uart",
    feature = "heltec_v4",
    feature = "elecrow_thinknode_m2",
    feature = "wio_tracker_l1",
    feature = "waveshare_rp2040_lora"
))]
pub mod simple_led;
