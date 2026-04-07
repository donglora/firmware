//! DongLoRa firmware — transparent LoRa radio over USB.
//!
//! Three async tasks communicate via static channels:
//!
//! ```text
//! host_task ──Command──► radio_task ──► SX1262
//!           ◄──Response──     │
//!                        StatusWatch
//!                             ▼
//!                       display_task (optional)
//! ```
//!
//! The host drives everything. The radio idles until commanded.

#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

#[cfg(not(test))]
mod board;
#[cfg(not(test))]
mod channel;
#[cfg(not(test))]
mod display;
mod protocol;
#[cfg(not(test))]
mod radio;
#[cfg(all(not(test), feature = "heltec_v3"))]
mod protocol_io;
#[cfg(all(not(test), feature = "heltec_v3"))]
mod uart;
#[cfg(all(not(test), not(feature = "heltec_v3")))]
mod usb;

#[cfg(not(test))]
use embassy_executor::Spawner;

#[cfg(not(test))]
use crate::board::LoRaBoard;
#[cfg(not(test))]
use crate::channel::{CommandChannel, DisplayCommandChannel, ResponseChannel, StatusWatch};

#[cfg(not(test))]
cfg_if::cfg_if! {
    if #[cfg(any(feature = "rak_wisblock_4631", feature = "wio_tracker_l1"))] {
        use defmt_rtt as _;
        use panic_probe as _;
    } else if #[cfg(any(feature = "heltec_v3", feature = "heltec_v4"))] {
        use esp_backtrace as _;
        use esp_println as _;
    }
}

#[cfg(not(test))]
static COMMANDS: CommandChannel = CommandChannel::new();
#[cfg(not(test))]
static RESPONSES: ResponseChannel = ResponseChannel::new();
#[cfg(not(test))]
static STATUS: StatusWatch = StatusWatch::new();
#[cfg(not(test))]
static DISPLAY_COMMANDS: DisplayCommandChannel = DisplayCommandChannel::new();

#[cfg(not(test))]
cfg_if::cfg_if! {
    if #[cfg(any(feature = "rak_wisblock_4631", feature = "wio_tracker_l1"))] {
        #[embassy_executor::main]
        async fn main(spawner: Spawner) {
            run(spawner).await;
        }
    } else if #[cfg(any(feature = "heltec_v3", feature = "heltec_v4"))] {
        #[esp_rtos::main]
        async fn main(spawner: Spawner) {
            run(spawner).await;
        }
    }
}

#[cfg(not(test))]
async fn run(spawner: Spawner) {
    let board = <board::Board as LoRaBoard>::init();
    let mac = board::Board::mac_address();
    let (radio, comm, display) = board.into_parts();

    let has_display = display.is_some();

    spawner
        .spawn(radio::radio_task(radio, &COMMANDS, &RESPONSES, &STATUS))
        .expect("spawn radio_task");

    cfg_if::cfg_if! {
        if #[cfg(feature = "heltec_v3")] {
            spawner
                .spawn(uart::uart_task(
                    comm,
                    &COMMANDS,
                    &RESPONSES,
                    &DISPLAY_COMMANDS,
                    has_display,
                    mac,
                ))
                .expect("spawn uart_task");
        } else {
            spawner
                .spawn(usb::usb_task(
                    comm,
                    &COMMANDS,
                    &RESPONSES,
                    &DISPLAY_COMMANDS,
                    has_display,
                    mac,
                ))
                .expect("spawn usb_task");
        }
    }

    if let Some(dp) = display {
        spawner
            .spawn(display::display_task(dp, &STATUS, &DISPLAY_COMMANDS))
            .expect("spawn display_task");
    }
}
