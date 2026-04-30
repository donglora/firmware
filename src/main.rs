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
#[cfg(all(not(test), feature = "debug-checkpoint"))]
mod debug_blink;
#[cfg(not(test))]
mod display;
#[cfg(not(test))]
mod driver;
#[cfg(not(test))]
mod hal;
#[cfg(not(test))]
mod host;
mod lbt;
mod protocol;
#[cfg(not(test))]
mod radio;

#[cfg(not(test))]
use embassy_executor::Spawner;

#[cfg(not(test))]
use crate::board::LoRaBoard;
#[cfg(not(test))]
use crate::channel::{CommandChannel, DisplayCommandChannel, OutboundChannel, RadioEventChannel};
#[cfg(not(test))]
use crate::protocol::{cap, Info, MAX_MCU_UID_LEN, MAX_RADIO_UID_LEN};

#[cfg(not(test))]
cfg_if::cfg_if! {
    if #[cfg(any(feature = "heltec_mesh_node_t114", feature = "rak_wisblock_4631", feature = "wio_tracker_l1", feature = "waveshare_rp2040_lora"))] {
        use defmt_rtt as _;
        use panic_probe as _;
    } else if #[cfg(any(feature = "heltec_v3", feature = "heltec_v3_uart", feature = "heltec_v4", feature = "elecrow_thinknode_m2", feature = "lilygo_tbeam_supreme", feature = "lilygo_tbeam"))] {
        use esp_backtrace as _;
        use esp_println as _;
        // Emits the IDF app descriptor into `.flash.appdesc`. espflash v4
        // refuses any ESP image that lacks it.
        esp_bootloader_esp_idf::esp_app_desc!();
    }
}

#[cfg(not(test))]
static COMMANDS: CommandChannel = CommandChannel::new();
#[cfg(not(test))]
static OUTBOUND: OutboundChannel = OutboundChannel::new();
#[cfg(not(test))]
static RADIO_EVENTS: RadioEventChannel = RadioEventChannel::new();
#[cfg(not(test))]
static DISPLAY_COMMANDS: DisplayCommandChannel = DisplayCommandChannel::new();
#[cfg(not(test))]
static mut INFO: Option<Info> = None;

#[cfg(not(test))]
cfg_if::cfg_if! {
    if #[cfg(any(feature = "heltec_mesh_node_t114", feature = "rak_wisblock_4631", feature = "wio_tracker_l1", feature = "waveshare_rp2040_lora"))] {
        #[embassy_executor::main]
        async fn main(spawner: Spawner) {
            run(spawner).await;
        }
    } else if #[cfg(any(feature = "heltec_v3", feature = "heltec_v3_uart", feature = "heltec_v4", feature = "elecrow_thinknode_m2", feature = "lilygo_tbeam_supreme", feature = "lilygo_tbeam"))] {
        #[esp_rtos::main]
        async fn main(spawner: Spawner) {
            run(spawner).await;
        }
    }
}

#[cfg(not(test))]
async fn run(spawner: Spawner) {
    let board = <board::Board as LoRaBoard>::init();

    // Read + log + clear POWER.RESETREAS before any other boot output so
    // the cause of the *previous* reset (if any) lands at the very top
    // of the RTT log. Critical for soak-debugging silent crash-reboots:
    // a self-rebooting firmware otherwise looks like the log just
    // restarted with no explanation. nRF52840-only.
    #[cfg(any(feature = "heltec_mesh_node_t114", feature = "rak_wisblock_4631", feature = "wio_tracker_l1"))]
    crate::hal::nrf52840::dump_reset_reason();

    let parts = board.into_parts();

    // Build the GET_INFO struct once at boot. This is the authoritative
    // snapshot of device capabilities reported over the wire.
    let info = build_info(parts.mac);
    // SAFETY: touched only here at boot, before any task is spawned.
    let info_ref: &'static Info = unsafe {
        INFO = Some(info);
        #[allow(static_mut_refs)]
        INFO.as_ref().expect("INFO just initialized")
    };

    spawner.spawn(
        radio::radio_task(parts.radio, &COMMANDS, &OUTBOUND, &RADIO_EVENTS, info_ref)
            .expect("spawn radio_task"),
    );

    spawner.spawn(
        host::host_task(
            parts.host,
            &COMMANDS,
            &OUTBOUND,
            &DISPLAY_COMMANDS,
            parts.display.is_some(),
            parts.mac,
        )
        .expect("spawn host_task"),
    );

    // Suppress "assigned but never read" warning on non-USB/UART path.
    let _ = info_ref;

    if let Some(dp) = parts.display {
        #[cfg(feature = "debug-checkpoint")]
        {
            // LED is taken over by the checkpoint blinker; the display
            // init struct is dropped unused for this debug build.
            let _ = dp;
            spawner
                .spawn(debug_blink::debug_blink_task(parts.led).expect("spawn debug_blink_task"));
        }
        #[cfg(not(feature = "debug-checkpoint"))]
        {
            #[allow(clippy::unit_arg)] // LedDriver is () for boards without LEDs
            spawner.spawn(
                display::display_task(dp, parts.led, &RADIO_EVENTS, &DISPLAY_COMMANDS)
                    .expect("spawn display_task"),
            );
        }
    }
}

#[cfg(not(test))]
fn build_info(mac: [u8; 6]) -> Info {
    let (fmin, fmax) = <board::Board as LoRaBoard>::freq_range_hz();
    let chip_id = <board::Board as LoRaBoard>::radio_chip_id();
    let (tx_min, tx_max) = board::Board::TX_POWER_RANGE;
    let supported_sf = <board::Board as LoRaBoard>::supported_sf_bitmap();
    let supported_bw = <board::Board as LoRaBoard>::supported_bw_bitmap();

    let mut mcu_uid = [0u8; MAX_MCU_UID_LEN];
    mcu_uid[..6].copy_from_slice(&mac);

    Info {
        proto_major: 1,
        proto_minor: 0,
        fw_major: parse_u8_or(env!("CARGO_PKG_VERSION_MAJOR"), 0),
        fw_minor: parse_u8_or(env!("CARGO_PKG_VERSION_MINOR"), 0),
        fw_patch: parse_u8_or(env!("CARGO_PKG_VERSION_PATCH"), 0),
        radio_chip_id: chip_id,
        capability_bitmap: cap::LORA | cap::CAD_BEFORE_TX,
        supported_sf_bitmap: supported_sf,
        supported_bw_bitmap: supported_bw,
        max_payload_bytes: crate::protocol::MAX_OTA_PAYLOAD as u16,
        // Real channel depths from channel.rs.
        rx_queue_capacity: 32,
        tx_queue_capacity: 1,
        freq_min_hz: fmin,
        freq_max_hz: fmax,
        tx_power_min_dbm: tx_min,
        tx_power_max_dbm: tx_max,
        mcu_uid_len: 6,
        mcu_uid,
        radio_uid_len: 0,
        radio_uid: [0u8; MAX_RADIO_UID_LEN],
    }
}

#[cfg(not(test))]
const fn parse_u8_or(s: &str, default: u8) -> u8 {
    // CARGO_PKG_VERSION_* strings are short decimal ascii. Parse them at
    // compile time via a tiny constant function instead of pulling in a
    // dependency.
    let bytes = s.as_bytes();
    let mut i = 0;
    let mut out: u32 = 0;
    while i < bytes.len() {
        let b = bytes[i];
        if b < b'0' || b > b'9' {
            return default;
        }
        out = out * 10 + (b - b'0') as u32;
        if out > u8::MAX as u32 {
            return default;
        }
        i += 1;
    }
    out as u8
}
