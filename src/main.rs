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
#[cfg(not(test))]
mod driver;
#[cfg(not(test))]
mod hal;
#[cfg(not(test))]
mod host;
mod protocol;
#[cfg(not(test))]
mod radio;

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
    } else if #[cfg(any(feature = "heltec_v3", feature = "heltec_v3_uart", feature = "heltec_v4"))] {
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
    } else if #[cfg(any(feature = "heltec_v3", feature = "heltec_v3_uart", feature = "heltec_v4"))] {
        #[esp_rtos::main]
        async fn main(spawner: Spawner) {
            run(spawner).await;
        }
    }
}

#[cfg(not(test))]
async fn run(spawner: Spawner) {
    let board = <board::Board as LoRaBoard>::init();
    let parts = board.into_parts();

    spawner
        .spawn(radio::radio_task(parts.radio, &COMMANDS, &RESPONSES, &STATUS))
        .expect("spawn radio_task");

    spawner
        .spawn(host::host_task(
            parts.host,
            &COMMANDS,
            &RESPONSES,
            &DISPLAY_COMMANDS,
            parts.display.is_some(),
            parts.mac,
        ))
        .expect("spawn host_task");

    if let Some(dp) = parts.display {
        spawner
            .spawn(display::display_task(dp, &STATUS, &DISPLAY_COMMANDS))
            .expect("spawn display_task");
    }
}
