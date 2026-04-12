//! Host communication: unified USB/UART transport.
//!
//! Exports a single `host_task` regardless of transport.
//! The cfg dispatch is contained entirely within this module.

pub mod framing;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "heltec_v3_uart", feature = "elecrow_thinknode_m2"))] {
        mod uart;
        pub use uart::host_task;
    } else {
        mod usb;
        pub use usb::host_task;
    }
}
