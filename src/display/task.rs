use embassy_executor::task;
use embassy_futures::select::{select3, Either3};
use embassy_time::Timer;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::BinaryColor;

use crate::board::DisplayParts;
use crate::channel::{DisplayCommand, DisplayCommandChannel, RadioState, RadioStatus, StatusWatch};

use crate::board::{Board, LoRaBoard};

use super::render::{self, RSSI_HISTORY_LEN};

const BOARD_NAME: &str = Board::NAME;

/// Duration per sparkline slot. 128 slots * 1s = ~2 minutes of history.
const SPARK_SLOT_MS: u64 = 1000;

/// Sentinel: no packet received in this slot. Below SX1262 sensitivity
/// floor (-120 dBm), so it cannot be confused with a real RSSI value.
const NO_SIGNAL: i16 = -121;

struct DisplayState {
    rssi_history: [i16; RSSI_HISTORY_LEN],
    tx_history: [bool; RSSI_HISTORY_LEN],
    rssi_count: usize,
    current_slot_rssi: i16,
    current_slot_tx: bool,
    display_on: bool,
    disconnected: bool,
    last_status: RadioStatus,
}

impl DisplayState {
    fn new() -> Self {
        Self {
            rssi_history: [NO_SIGNAL; RSSI_HISTORY_LEN],
            tx_history: [false; RSSI_HISTORY_LEN],
            rssi_count: 0,
            current_slot_rssi: NO_SIGNAL,
            current_slot_tx: false,
            display_on: true,
            disconnected: true,
            last_status: RadioStatus::default(),
        }
    }

    /// Record an RSSI sample in the current time slot (keep best).
    fn record_rssi(&mut self, rssi: i16) {
        if self.current_slot_rssi == NO_SIGNAL || rssi > self.current_slot_rssi {
            self.current_slot_rssi = rssi;
        }
    }

    /// Mark the current time slot as having a transmit.
    fn record_tx(&mut self) {
        self.current_slot_tx = true;
    }

    /// Advance to the next time slot, committing the current slot's data.
    fn advance_slot(&mut self) {
        let idx = self.rssi_count % RSSI_HISTORY_LEN;
        self.rssi_history[idx] = self.current_slot_rssi;
        self.tx_history[idx] = self.current_slot_tx;
        self.rssi_count += 1;
        self.current_slot_rssi = NO_SIGNAL;
        self.current_slot_tx = false;
    }

    /// Whether the active dashboard should be shown (RX or TX mode).
    fn is_active(&self) -> bool {
        matches!(
            self.last_status.state,
            RadioState::Receiving | RadioState::Transmitting
        )
    }
}

/// Render the appropriate screen for the current state into the display buffer.
fn render_current(
    display: &mut impl DrawTarget<Color = BinaryColor>,
    state: &DisplayState,
    board: &render::BoardInfo<'_>,
) {
    if state.is_active() {
        render::dashboard(
            display,
            &state.last_status,
            &state.rssi_history,
            &state.tx_history,
            state.rssi_count,
            state.current_slot_rssi,
            state.current_slot_tx,
            board,
        );
    } else {
        render::splash(display, board);
    }
}

#[task]
pub async fn display_task(
    parts: DisplayParts,
    status: &'static StatusWatch,
    display_commands: &'static DisplayCommandChannel,
) {
    // Format MAC address as "XX:XX:XX:XX:XX:XX"
    let mut mac_str: heapless::String<18> = heapless::String::new();
    let m = parts.mac;
    let _ = core::fmt::Write::write_fmt(
        &mut mac_str,
        format_args!(
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            m[0], m[1], m[2], m[3], m[4], m[5]
        ),
    );

    let board_info = render::BoardInfo {
        name: BOARD_NAME,
        version: env!("CARGO_PKG_VERSION"),
        mac: &mac_str,
    };

    cfg_if::cfg_if! {
        if #[cfg(feature = "wio_tracker_l1")] {
            let mut display = crate::driver::sh1106::Sh1106::new(parts.i2c, 0x3D);

            Timer::after_millis(100).await;

            if display.init().await.is_err() {
                defmt::error!("SH1106 init failed — retrying");
                Timer::after_millis(500).await;
                if display.init().await.is_err() {
                    defmt::error!("SH1106 init failed twice, giving up");
                    return;
                }
            }
            if display.set_brightness(0xFF).await.is_err() {
                defmt::warn!("display brightness set failed");
            }
        } else {
            use ssd1306::mode::DisplayConfigAsync;
            use ssd1306::prelude::DisplayRotation;
            use ssd1306::size::DisplaySize128x64;
            use ssd1306::{I2CDisplayInterface, Ssd1306Async};

            let interface = I2CDisplayInterface::new(parts.i2c);
            let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
                .into_buffered_graphics_mode();

            Timer::after_millis(100).await;

            if display.init().await.is_err() {
                defmt::error!("SSD1306 init failed — retrying");
                Timer::after_millis(500).await;
                if display.init().await.is_err() {
                    defmt::error!("SSD1306 init failed twice, giving up");
                    return;
                }
            }
            if display
                .set_brightness(ssd1306::prelude::Brightness::BRIGHTEST)
                .await
                .is_err()
            {
                defmt::warn!("display brightness set failed");
            }
        }
    }

    let mut state = DisplayState::new();
    let Some(mut receiver) = status.receiver() else {
        defmt::error!("no watch receiver available for display");
        return;
    };

    // Show splash/waiting screen
    render::splash(&mut display, &board_info);
    let _ = display.flush().await;

    loop {
        match select3(
            receiver.changed(),
            display_commands.receive(),
            Timer::after_millis(SPARK_SLOT_MS),
        )
        .await
        {
            Either3::First(radio_status) => {
                if state.disconnected {
                    continue;
                }
                if let Some(rssi) = radio_status.last_rssi {
                    if radio_status.rx_count != state.last_status.rx_count {
                        state.record_rssi(rssi);
                    }
                }
                if radio_status.tx_count != state.last_status.tx_count {
                    state.record_tx();
                }
                state.last_status = radio_status;

                if state.display_on {
                    render_current(&mut display, &state, &board_info);
                    let _ = display.flush().await;
                }
            }
            Either3::Second(cmd) => match cmd {
                DisplayCommand::Off => {
                    state.disconnected = false;
                    state.display_on = false;
                    render::blank(&mut display);
                    let _ = display.flush().await;
                }
                DisplayCommand::On => {
                    state.disconnected = false;
                    state.display_on = true;
                    if let Some(s) = receiver.try_get() {
                        state.last_status = s;
                    }
                    render_current(&mut display, &state, &board_info);
                    let _ = display.flush().await;
                }
                DisplayCommand::Reset => {
                    state = DisplayState::new();
                    render::splash(&mut display, &board_info);
                    let _ = display.flush().await;
                }
            },
            Either3::Third(()) => {
                // Timer tick: advance sparkline slot
                state.advance_slot();
                if state.display_on && state.is_active() {
                    render_current(&mut display, &state, &board_info);
                    let _ = display.flush().await;
                }
            }
        }
    }
}
