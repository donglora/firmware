//! OLED display dashboard, driven by a hsmc statechart.
//!
//! ```text
//! Display
//! ├── Off            display blank, LED off
//! └── On
//!     ├── Splash                         radio idle / no activity
//!     │   ├── LearnMore   (default)      QR code + "donglora.com"
//!     │   └── Info                       firmware version + MAC
//!     └── Dashboard
//!         ├── Listening   (default)      radio in RX
//!         └── Transmitting               radio in TX
//! ```
//!
//! The two Splash substates alternate on a 5 s timer via `on(after …)`
//! transitions pointing at each other. No ticker, no timer task — hsmc
//! handles the flip entirely inside the statechart.
//!
//! `CmdReset` (sent by the host task on USB disconnect) runs a reset
//! action and re-enters `Splash`. This is order-idempotent with the
//! `RadioIdle` that arrives from the radio at the same moment: whichever
//! event the machine sees first, we end up in `Splash.LearnMore` with a
//! fresh timer and a cleared run state.
//!
//! Display subscribes to [`RadioEventChannel`] (published by the radio task)
//! and [`DisplayCommandChannel`] (published by the host task). Two trivial
//! forwarder futures translate each source into [`DisplayEvent`]s on the
//! machine's own channel — the machine is the single source of truth.

use core::time::Duration;

use embassy_executor::task;
use embassy_futures::join::join3;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use hsmc::statechart;

use crate::board::{self, Board, DisplayDriver, DisplayParts, LedDriver, LoRaBoard, RgbLed};
use crate::channel::{DisplayCommand, DisplayCommandChannel, RadioEvent, RadioEventChannel};
use crate::protocol::RadioConfig;

use render::{BoardInfo, RSSI_HISTORY_LEN};

const BOARD_NAME: &str = Board::NAME;
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Sentinel: no packet received in this slot. Below SX1262 sensitivity
/// floor (-120 dBm), so it cannot be confused with a real RSSI value.
const NO_SIGNAL: i16 = -121;

/// Which dashboard badge is in view (drives the RX/TX indicator only).
#[derive(Debug, Clone, Copy)]
pub enum Badge {
    Rx,
    Tx,
}

/// Fine-grained events driving the `Display` statechart.
#[derive(Debug, Clone)]
pub enum DisplayEvent {
    CmdOn,
    CmdOff,
    CmdReset,
    /// Radio moved into RX mode.
    EnteredRx,
    /// Radio moved into TX mode.
    EnteredTx,
    /// Radio returned to Idle.
    RadioIdle,
    /// A packet was received (data only — does not imply a mode change).
    PacketRx {
        rssi: i16,
        snr: Option<i16>,
    },
    /// A packet was transmitted (data only — does not imply a mode change).
    PacketTx,
    ConfigChanged(RadioConfig),
}

impl From<RadioEvent> for DisplayEvent {
    fn from(ev: RadioEvent) -> Self {
        match ev {
            RadioEvent::EnteredRx => DisplayEvent::EnteredRx,
            RadioEvent::EnteredTx => DisplayEvent::EnteredTx,
            RadioEvent::Idle => DisplayEvent::RadioIdle,
            RadioEvent::PacketRx { rssi, snr } => DisplayEvent::PacketRx { rssi, snr },
            RadioEvent::PacketTx => DisplayEvent::PacketTx,
            RadioEvent::ConfigChanged(cfg) => DisplayEvent::ConfigChanged(cfg),
        }
    }
}

static DISPLAY_EVENTS: Channel<CriticalSectionRawMutex, DisplayEvent, 8> = Channel::new();

// ── Context ────────────────────────────────────────────────────────

/// Peripherals the display task owns for its entire lifetime.
struct Peripherals {
    display: DisplayDriver,
    led: LedDriver,
    mac_str: heapless::String<18>,
}

/// Everything `CmdReset` should clear — the dashboard's "view" of the radio.
struct RunState {
    rssi_history: [i16; RSSI_HISTORY_LEN],
    tx_history: [bool; RSSI_HISTORY_LEN],
    rssi_count: usize,
    current_slot_rssi: i16,
    current_slot_tx: bool,
    config: Option<RadioConfig>,
    rx_count: u32,
    tx_count: u32,
    last_rssi: Option<i16>,
    last_snr: Option<i16>,
}

impl RunState {
    fn new() -> Self {
        Self {
            rssi_history: [NO_SIGNAL; RSSI_HISTORY_LEN],
            tx_history: [false; RSSI_HISTORY_LEN],
            rssi_count: 0,
            current_slot_rssi: NO_SIGNAL,
            current_slot_tx: false,
            config: None,
            rx_count: 0,
            tx_count: 0,
            last_rssi: None,
            last_snr: None,
        }
    }

    fn record_rssi(&mut self, rssi: i16) {
        if self.current_slot_rssi == NO_SIGNAL || rssi > self.current_slot_rssi {
            self.current_slot_rssi = rssi;
        }
    }

    fn advance_slot(&mut self) {
        let idx = self.rssi_count % RSSI_HISTORY_LEN;
        self.rssi_history[idx] = self.current_slot_rssi;
        self.tx_history[idx] = self.current_slot_tx;
        self.rssi_count += 1;
        self.current_slot_rssi = NO_SIGNAL;
        self.current_slot_tx = false;
    }
}

pub struct DisplayContext {
    peripherals: Peripherals,
    run: RunState,
}

impl DisplayContext {
    async fn flush_info(&mut self) {
        let Peripherals {
            display, mac_str, ..
        } = &mut self.peripherals;
        let info = BoardInfo {
            name: BOARD_NAME,
            version: VERSION,
            mac: mac_str,
        };
        render::info(display, &info);
        let _ = display.flush().await;
    }

    async fn flush_learn_more(&mut self) {
        let display = &mut self.peripherals.display;
        render::learn_more(display);
        let _ = display.flush().await;
    }

    async fn flush_dashboard(&mut self, badge: Badge) {
        let Peripherals {
            display, mac_str, ..
        } = &mut self.peripherals;
        let info = BoardInfo {
            name: BOARD_NAME,
            version: VERSION,
            mac: mac_str,
        };
        render::dashboard(
            display,
            badge,
            self.run.config,
            self.run.rx_count,
            self.run.tx_count,
            self.run.last_rssi,
            self.run.last_snr,
            &self.run.rssi_history,
            &self.run.tx_history,
            self.run.rssi_count,
            self.run.current_slot_rssi,
            self.run.current_slot_tx,
            &info,
        );
        let _ = display.flush().await;
    }

    async fn flush_blank(&mut self) {
        render::blank(&mut self.peripherals.display);
        let _ = self.peripherals.display.flush().await;
    }

    // Boards without an LED use `LedDriver = ()`; the `RgbLed for ()` impl
    // makes `set_rgb` a noop returning `()`. Clippy's unit_arg lint
    // misreads method calls on unit values as "passing a unit value to a
    // function" — silence it here where the noop is intentional.
    #[allow(clippy::unit_arg)]
    async fn led_off(&mut self) {
        self.peripherals.led.set_rgb(0, 0, 0).await;
    }

    #[allow(clippy::unit_arg)]
    async fn led_blip(&mut self, r: u8, g: u8, b: u8) {
        self.peripherals.led.set_rgb(r, g, b).await;
        Timer::after_millis(50).await;
        self.peripherals.led.set_rgb(0, 0, 0).await;
    }
}

// ── Statechart ─────────────────────────────────────────────────────

statechart! {
    Display {
        context: DisplayContext;
        events: DisplayEvent;
        default(On);

        state Off {
            entry: blank_display, turn_led_off;
            on(CmdOn) => On;
            // Stay off, but scrub any stale run state so reconnecting
            // doesn't surface old counters/sparkline.
            on(CmdReset) => reset_run_state;
        }

        state On {
            default(Splash);
            on(CmdOff) => Off;
            // `CmdReset` + `RadioIdle` race on USB disconnect; this handler
            // makes either ordering land in the same place. Reset the run
            // state, turn off any blink-in-progress, and re-enter Splash —
            // descend_defaults lands us in LearnMore with a fresh 5 s timer.
            on(CmdReset) => reset_run_state, turn_led_off, Splash;
            // Config can arrive before the dashboard is up; cache it anywhere.
            on(ConfigChanged(cfg: RadioConfig)) => save_new_config;

            state Splash {
                default(LearnMore);
                on(EnteredRx) => Listening;
                on(EnteredTx) => Transmitting;

                state LearnMore {
                    entry: paint_learn_more;
                    on(after Duration::from_secs(5)) => Info;
                }

                state Info {
                    entry: paint_info;
                    on(after Duration::from_secs(5)) => LearnMore;
                }
            }

            state Dashboard {
                default(Listening);
                on(RadioIdle) => Splash;

                // Packet events are pure data + one-shot LED feedback.
                on(PacketRx { rssi: i16, snr: Option<i16> }) => record_rx_packet, blink_rx_led;
                on(PacketTx) => record_tx_packet, blink_tx_led;

                state Listening {
                    entry: paint_listening;
                    on(EnteredTx) => Transmitting;
                    on(every Duration::from_millis(1000)) => advance_sparkline, paint_listening;
                }

                state Transmitting {
                    entry: paint_transmitting;
                    on(EnteredRx) => Listening;
                    on(every Duration::from_millis(1000)) => advance_sparkline, paint_transmitting;
                }
            }
        }
    }
}

// ── Actions ────────────────────────────────────────────────────────

/// Map SNR (dB) to LED brightness (4..64). Stronger signal = brighter.
fn snr_brightness(snr: Option<i16>) -> u8 {
    let snr = snr.unwrap_or(-10);
    let clamped = snr.clamp(-20, 15) as i32;
    ((clamped + 20) * 60 / 35 + 4) as u8
}

impl DisplayActions for DisplayActionContext<'_> {
    async fn blank_display(&mut self) {
        self.flush_blank().await;
    }

    async fn turn_led_off(&mut self) {
        self.led_off().await;
    }

    async fn reset_run_state(&mut self) {
        self.run = RunState::new();
    }

    async fn paint_info(&mut self) {
        self.flush_info().await;
    }

    async fn paint_learn_more(&mut self) {
        self.flush_learn_more().await;
    }

    async fn paint_listening(&mut self) {
        self.flush_dashboard(Badge::Rx).await;
    }

    async fn paint_transmitting(&mut self) {
        self.flush_dashboard(Badge::Tx).await;
    }

    async fn record_rx_packet(&mut self, rssi: i16, snr: Option<i16>) {
        self.run.rx_count = self.run.rx_count.wrapping_add(1);
        self.run.last_rssi = Some(rssi);
        self.run.last_snr = snr;
        self.run.record_rssi(rssi);
    }

    async fn record_tx_packet(&mut self) {
        self.run.tx_count = self.run.tx_count.wrapping_add(1);
        self.run.current_slot_tx = true;
    }

    async fn blink_rx_led(&mut self, _rssi: i16, snr: Option<i16>) {
        let brightness = snr_brightness(snr);
        self.led_blip(0, brightness, 0).await;
    }

    async fn blink_tx_led(&mut self) {
        self.led_blip(32, 0, 0).await;
    }

    async fn save_new_config(&mut self, cfg: RadioConfig) {
        self.run.config = Some(cfg);
    }

    async fn advance_sparkline(&mut self) {
        self.run.advance_slot();
    }
}

// ── Task entry point ───────────────────────────────────────────────

#[task]
pub async fn display_task(
    parts: DisplayParts,
    mut led: LedDriver,
    radio_events: &'static RadioEventChannel,
    display_commands: &'static DisplayCommandChannel,
) {
    let mut mac_str: heapless::String<18> = heapless::String::new();
    let m = Board::mac_address();
    let _ = core::fmt::Write::write_fmt(
        &mut mac_str,
        format_args!(
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            m[0], m[1], m[2], m[3], m[4], m[5]
        ),
    );

    let Some(display) = board::create_display(parts.i2c).await else {
        defmt::error!("display init failed, running LED-only loop");
        loop {
            match display_commands.receive().await {
                DisplayCommand::Reset | DisplayCommand::Off => {
                    led.set_rgb(0, 0, 0).await;
                }
                DisplayCommand::On => {}
            }
        }
    };

    let ctx = DisplayContext {
        peripherals: Peripherals {
            display,
            led,
            mac_str,
        },
        run: RunState::new(),
    };

    let mut machine = Display::new(ctx, &DISPLAY_EVENTS);
    let sender = DisplaySender::from_channel(&DISPLAY_EVENTS);

    let run = async {
        let _ = machine.run().await;
    };

    let command_forwarder = async {
        loop {
            let ev = match display_commands.receive().await {
                DisplayCommand::On => DisplayEvent::CmdOn,
                DisplayCommand::Off => DisplayEvent::CmdOff,
                DisplayCommand::Reset => DisplayEvent::CmdReset,
            };
            let _ = sender.send(ev).await;
        }
    };

    let radio_event_forwarder = async {
        loop {
            let ev = radio_events.receive().await;
            let _ = sender.send(ev.into()).await;
        }
    };

    join3(run, command_forwarder, radio_event_forwarder).await;
}

// ── Rendering ──────────────────────────────────────────────────────

mod render {
    use core::fmt::Write;

    use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X15_BOLD};
    use embedded_graphics::mono_font::MonoTextStyle;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::prelude::*;
    use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
    use embedded_graphics::text::renderer::CharacterStyle;
    use embedded_graphics::text::{Alignment, Text};
    use heapless::String;

    use super::Badge;
    use crate::protocol::{Bandwidth, RadioConfig};

    pub const RSSI_HISTORY_LEN: usize = 128;

    const CHAR_W: i32 = 6;
    const FONT_H: i32 = 10;
    const MODE_BOX_W: i32 = 2 * CHAR_W + 2;

    const RSSI_MIN: i16 = -120;
    const RSSI_MAX: i16 = 0;

    /// Hand-authored 64 × 64 pixel-perfect QR bitmap for the splash
    /// "learn more" screen. 1 bit/pixel, MSB-first, row-major. Bit `1` is
    /// "light" (OLED pixel on), bit `0` is "dark" (OLED pixel off), so
    /// when drawn with `BinaryColor::On` = lit, the QR appears with the
    /// traditional dark-on-light look including a proper quiet zone.
    const QR_WIDTH_PX: u32 = 64;
    const QR_BITMAP: [u8; 512] = [
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x07, 0x9F,
        0xFE, 0x60, 0x00, 0x7F, 0xFE, 0x00, 0x07, 0x9F, 0xFE, 0x60, 0x00, 0x7F, 0xFE, 0x7F, 0xE6,
        0x18, 0x19, 0xE7, 0xFE, 0x7F, 0xFE, 0x7F, 0xE6, 0x18, 0x19, 0xE7, 0xFE, 0x7F, 0xFE, 0x60,
        0x67, 0x81, 0xE6, 0x66, 0x06, 0x7F, 0xFE, 0x60, 0x67, 0x81, 0xE6, 0x66, 0x06, 0x7F, 0xFE,
        0x60, 0x66, 0x19, 0x87, 0xE6, 0x06, 0x7F, 0xFE, 0x60, 0x66, 0x19, 0x87, 0xE6, 0x06, 0x7F,
        0xFE, 0x60, 0x67, 0xE6, 0x1F, 0xE6, 0x06, 0x7F, 0xFE, 0x60, 0x67, 0xE6, 0x1F, 0xE6, 0x06,
        0x7F, 0xFE, 0x7F, 0xE6, 0x67, 0x80, 0x67, 0xFE, 0x7F, 0xFE, 0x7F, 0xE6, 0x67, 0x80, 0x67,
        0xFE, 0x7F, 0xFE, 0x00, 0x06, 0x66, 0x66, 0x60, 0x00, 0x7F, 0xFE, 0x00, 0x06, 0x66, 0x66,
        0x60, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x61, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
        0x61, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x60, 0x06, 0x01, 0x99, 0x99, 0xFF, 0xFE, 0x00, 0x60,
        0x06, 0x01, 0x99, 0x99, 0xFF, 0xFE, 0x01, 0x98, 0x1F, 0x9E, 0x79, 0xF9, 0xFF, 0xFE, 0x01,
        0x98, 0x1F, 0x9E, 0x79, 0xF9, 0xFF, 0xFF, 0xE0, 0x66, 0x18, 0x61, 0x9F, 0x98, 0x7F, 0xFF,
        0xE0, 0x66, 0x18, 0x61, 0x9F, 0x98, 0x7F, 0xFF, 0x98, 0x1E, 0x01, 0xE7, 0x99, 0xFE, 0x7F,
        0xFF, 0x98, 0x1E, 0x01, 0xE7, 0x99, 0xFE, 0x7F, 0xFF, 0xE1, 0xE6, 0x19, 0xF8, 0x06, 0x60,
        0x7F, 0xFF, 0xE1, 0xE6, 0x19, 0xF8, 0x06, 0x60, 0x7F, 0xFE, 0x19, 0xF8, 0x06, 0x1E, 0x79,
        0x99, 0xFF, 0xFE, 0x19, 0xF8, 0x06, 0x1E, 0x79, 0x99, 0xFF, 0xFE, 0x7E, 0x01, 0xFF, 0x80,
        0x18, 0x18, 0x7F, 0xFE, 0x7E, 0x01, 0xFF, 0x80, 0x18, 0x18, 0x7F, 0xFE, 0x7E, 0x19, 0x9E,
        0x1E, 0x78, 0x7E, 0x7F, 0xFE, 0x7E, 0x19, 0x9E, 0x1E, 0x78, 0x7E, 0x7F, 0xFE, 0x60, 0x06,
        0x66, 0x66, 0x00, 0x67, 0xFF, 0xFE, 0x60, 0x06, 0x66, 0x66, 0x00, 0x67, 0xFF, 0xFF, 0xFF,
        0xFE, 0x67, 0xF8, 0x7E, 0x1F, 0xFF, 0xFF, 0xFF, 0xFE, 0x67, 0xF8, 0x7E, 0x1F, 0xFF, 0xFE,
        0x00, 0x06, 0x00, 0x7E, 0x66, 0x60, 0x7F, 0xFE, 0x00, 0x06, 0x00, 0x7E, 0x66, 0x60, 0x7F,
        0xFE, 0x7F, 0xE7, 0x99, 0xE6, 0x7E, 0x1E, 0x7F, 0xFE, 0x7F, 0xE7, 0x99, 0xE6, 0x7E, 0x1E,
        0x7F, 0xFE, 0x60, 0x66, 0x61, 0xF8, 0x00, 0x60, 0x7F, 0xFE, 0x60, 0x66, 0x61, 0xF8, 0x00,
        0x60, 0x7F, 0xFE, 0x60, 0x66, 0x06, 0x01, 0x86, 0x00, 0x7F, 0xFE, 0x60, 0x66, 0x06, 0x01,
        0x86, 0x00, 0x7F, 0xFE, 0x60, 0x66, 0x67, 0x86, 0x7F, 0x86, 0x7F, 0xFE, 0x60, 0x66, 0x67,
        0x86, 0x7F, 0x86, 0x7F, 0xFE, 0x7F, 0xE6, 0x66, 0x18, 0x00, 0x1E, 0x7F, 0xFE, 0x7F, 0xE6,
        0x66, 0x18, 0x00, 0x1E, 0x7F, 0xFE, 0x00, 0x06, 0x66, 0x67, 0x98, 0x00, 0x7F, 0xFE, 0x00,
        0x06, 0x66, 0x67, 0x98, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF,
    ];

    pub struct BoardInfo<'a> {
        pub name: &'a str,
        pub version: &'a str,
        pub mac: &'a str,
    }

    #[allow(clippy::too_many_arguments)]
    pub fn dashboard(
        target: &mut impl DrawTarget<Color = BinaryColor>,
        badge: Badge,
        config: Option<RadioConfig>,
        rx_count: u32,
        tx_count: u32,
        last_rssi: Option<i16>,
        last_snr: Option<i16>,
        rssi_history: &[i16; RSSI_HISTORY_LEN],
        tx_history: &[bool; RSSI_HISTORY_LEN],
        rssi_count: usize,
        current_slot_rssi: i16,
        current_slot_tx: bool,
        board: &BoardInfo,
    ) {
        let bb = target.bounding_box();
        let w = bb.size.width as i32;
        let h = bb.size.height as i32;
        let title_x = MODE_BOX_W + 1;
        let title_w = w - title_x;
        let header_h = 2 * FONT_H;
        let sep1_y = header_h + 3;
        let info_y = sep1_y + 1;
        let sep2_y = info_y + 2 * FONT_H + 3;
        let spark_top = sep2_y + 2;
        let spark_h = h - spark_top;
        let visible_bars = ((w / 2) as usize).min(RSSI_HISTORY_LEN);

        let _ = target.clear(BinaryColor::Off);
        let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let mut buf: String<32> = String::new();

        let fill = PrimitiveStyle::with_fill(BinaryColor::On);
        let mut inv_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::Off);
        inv_style.set_background_color(Some(BinaryColor::On));

        match badge {
            Badge::Rx => {
                Rectangle::new(
                    Point::new(0, 0),
                    Size::new(MODE_BOX_W as u32, FONT_H as u32),
                )
                .into_styled(fill)
                .draw(target)
                .ok();
                Text::new("RX", Point::new(1, FONT_H - 1), inv_style)
                    .draw(target)
                    .ok();
            }
            Badge::Tx => {
                Rectangle::new(
                    Point::new(0, FONT_H),
                    Size::new(MODE_BOX_W as u32, FONT_H as u32),
                )
                .into_styled(fill)
                .draw(target)
                .ok();
                Text::new("TX", Point::new(1, 2 * FONT_H - 1), inv_style)
                    .draw(target)
                    .ok();
            }
        }

        Line::new(Point::new(MODE_BOX_W, 0), Point::new(MODE_BOX_W, sep1_y))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(target)
            .ok();

        let title_center_x = title_x + title_w / 2;

        buf.clear();
        let _ = write!(buf, "DongLoRa v{}", board.version);
        Text::with_alignment(
            &buf,
            Point::new(title_center_x, FONT_H - 1),
            style,
            Alignment::Center,
        )
        .draw(target)
        .ok();

        if let Some(cfg) = config {
            buf.clear();
            let freq_mhz = cfg.freq_hz / 1_000_000;
            let freq_khz = (cfg.freq_hz % 1_000_000) / 1_000;
            let bw_str = match cfg.bw {
                Bandwidth::Khz7 => "7.8",
                Bandwidth::Khz10 => "10.4",
                Bandwidth::Khz15 => "15.6",
                Bandwidth::Khz20 => "20.8",
                Bandwidth::Khz31 => "31.2",
                Bandwidth::Khz41 => "41.7",
                Bandwidth::Khz62 => "62.5",
                Bandwidth::Khz125 => "125",
                Bandwidth::Khz250 => "250",
                Bandwidth::Khz500 => "500",
            };
            let _ = write!(
                buf,
                "{}.{:03}/{}/{}/{}",
                freq_mhz, freq_khz, bw_str, cfg.sf, cfg.cr
            );
            Text::with_alignment(
                &buf,
                Point::new(title_center_x, 2 * FONT_H - 1),
                style,
                Alignment::Center,
            )
            .draw(target)
            .ok();
        }

        Line::new(Point::new(0, sep1_y), Point::new(w - 1, sep1_y))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(target)
            .ok();

        let center_x = w / 2;

        buf.clear();
        let _ = write!(
            buf,
            "RX:{} TX:{}",
            compact_count(rx_count),
            compact_count(tx_count)
        );
        Text::with_alignment(
            &buf,
            Point::new(center_x, info_y + FONT_H - 1),
            style,
            Alignment::Center,
        )
        .draw(target)
        .ok();

        buf.clear();
        match (last_rssi, last_snr) {
            (Some(rssi), Some(snr)) => {
                let _ = write!(buf, "RSSI:{}dBm SNR:{}dB", rssi, snr);
            }
            (Some(rssi), None) => {
                let _ = write!(buf, "RSSI:{}dBm", rssi);
            }
            _ => {
                let _ = write!(buf, "No signal");
            }
        }
        Text::with_alignment(
            &buf,
            Point::new(center_x, info_y + 2 * FONT_H - 1),
            style,
            Alignment::Center,
        )
        .draw(target)
        .ok();

        Line::new(Point::new(0, sep2_y), Point::new(w - 1, sep2_y))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(target)
            .ok();

        if spark_h > 0 {
            rssi_sparkline(
                target,
                rssi_history,
                tx_history,
                rssi_count,
                current_slot_rssi,
                current_slot_tx,
                spark_top,
                spark_h,
                visible_bars,
            );
        }
    }

    pub fn info(target: &mut impl DrawTarget<Color = BinaryColor>, board: &BoardInfo) {
        let bb = target.bounding_box();
        let w = bb.size.width as i32;

        let _ = target.clear(BinaryColor::Off);
        let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let title_style = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);
        let mut buf: String<32> = String::new();

        let center_x = w / 2;

        Text::new("DongLoRa", Point::new(4, 15), title_style)
            .draw(target)
            .ok();
        buf.clear();
        let _ = write!(buf, "v{}", board.version);
        Text::with_alignment(&buf, Point::new(w - 2, 15), style, Alignment::Right)
            .draw(target)
            .ok();

        Text::with_alignment(
            board.name,
            Point::new(center_x, 28),
            style,
            Alignment::Center,
        )
        .draw(target)
        .ok();

        Text::with_alignment(
            board.mac,
            Point::new(center_x, 41),
            style,
            Alignment::Center,
        )
        .draw(target)
        .ok();

        Text::with_alignment(
            "Waiting for host...",
            Point::new(center_x, 54),
            style,
            Alignment::Center,
        )
        .draw(target)
        .ok();
    }

    /// Alternate splash screen: 64 × 64 pixel-perfect QR bitmap on the
    /// right (with its own quiet zone baked in), centered text column on
    /// the left inviting the user to visit the URL.
    pub fn learn_more(target: &mut impl DrawTarget<Color = BinaryColor>) {
        use embedded_graphics::image::{Image, ImageRaw};
        use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_7X14_BOLD};

        let bb = target.bounding_box();
        let w = bb.size.width as i32;

        let _ = target.clear(BinaryColor::Off);

        // ── QR code: flush to the right edge, 1:1 pixel rendering ───
        let qr_x = w - QR_WIDTH_PX as i32;
        let qr_raw: ImageRaw<BinaryColor> = ImageRaw::new(&QR_BITMAP, QR_WIDTH_PX);
        Image::new(&qr_raw, Point::new(qr_x, 0)).draw(target).ok();

        // ── Text column (everything left of the QR) ─────────────────
        let col_w = qr_x;
        let col_center = col_w / 2;

        let title_style = MonoTextStyle::new(&FONT_7X14_BOLD, BinaryColor::On);
        let text_style = MonoTextStyle::new(&FONT_5X8, BinaryColor::On);

        // Layout on a 64 px display:
        //   blank
        //   DongLoRa        baseline 22
        //   blank
        //   Learn more:     baseline 42
        //   donglora.com    baseline 52
        //   blank

        Text::with_alignment(
            "DongLoRa",
            Point::new(col_center, 22),
            title_style,
            Alignment::Center,
        )
        .draw(target)
        .ok();

        Text::with_alignment(
            "Learn more:",
            Point::new(col_center, 42),
            text_style,
            Alignment::Center,
        )
        .draw(target)
        .ok();

        Text::with_alignment(
            "donglora.com",
            Point::new(col_center, 52),
            text_style,
            Alignment::Center,
        )
        .draw(target)
        .ok();
    }

    fn compact_count(n: u32) -> String<10> {
        let mut s: String<10> = String::new();
        if n > 9_999_999 {
            let _ = write!(s, "{}M", n / 1_000_000);
        } else if n > 999_999 {
            let _ = write!(s, "{}k", n / 1_000);
        } else {
            let _ = write!(s, "{}", n);
        }
        s
    }

    #[allow(clippy::too_many_arguments)]
    fn rssi_sparkline(
        target: &mut impl DrawTarget<Color = BinaryColor>,
        history: &[i16; RSSI_HISTORY_LEN],
        tx_history: &[bool; RSSI_HISTORY_LEN],
        count: usize,
        current_rssi: i16,
        current_tx: bool,
        spark_top: i32,
        spark_h: i32,
        visible_bars: usize,
    ) {
        let live = current_rssi > RSSI_MIN || current_tx;
        let committed = count.min(RSSI_HISTORY_LEN);
        let hist_slots = if live {
            committed.min(visible_bars.saturating_sub(1))
        } else {
            committed.min(visible_bars)
        };
        let total = hist_slots + if live { 1 } else { 0 };

        if total == 0 {
            return;
        }

        let fill = PrimitiveStyle::with_fill(BinaryColor::On);

        for i in 0..hist_slots {
            let idx = if count <= RSSI_HISTORY_LEN {
                let effective_bars = if live { visible_bars - 1 } else { visible_bars };
                i + committed.saturating_sub(effective_bars)
            } else {
                let start = count - RSSI_HISTORY_LEN;
                let skip = RSSI_HISTORY_LEN.saturating_sub(if live {
                    visible_bars - 1
                } else {
                    visible_bars
                });
                (start + skip + i) % RSSI_HISTORY_LEN
            };
            let is_tx = tx_history[idx];
            let rssi = history[idx];

            if let Some(bar_h) = bar_height(rssi, is_tx, spark_h) {
                let x = (visible_bars - total + i) as i32 * 2;
                draw_bar(target, x, bar_h, is_tx, spark_top, spark_h, &fill);
            }
        }

        if live {
            if let Some(bar_h) = bar_height(current_rssi, current_tx, spark_h) {
                let x = (visible_bars - 1) as i32 * 2;
                draw_bar(target, x, bar_h, current_tx, spark_top, spark_h, &fill);
            }
        }
    }

    fn bar_height(rssi: i16, is_tx: bool, spark_h: i32) -> Option<i32> {
        let h = if rssi <= RSSI_MIN {
            if is_tx {
                spark_h / 3
            } else {
                return None;
            }
        } else {
            let clamped = rssi.clamp(RSSI_MIN, RSSI_MAX);
            ((clamped - RSSI_MIN) as i32 * spark_h) / (RSSI_MAX - RSSI_MIN) as i32
        };
        if h == 0 {
            None
        } else {
            Some(h)
        }
    }

    fn draw_bar(
        target: &mut impl DrawTarget<Color = BinaryColor>,
        x: i32,
        bar_h: i32,
        is_tx: bool,
        spark_top: i32,
        spark_h: i32,
        fill: &PrimitiveStyle<BinaryColor>,
    ) {
        let y = spark_top + spark_h - bar_h;
        if is_tx {
            for row in 0..bar_h {
                if row % 2 == 0 {
                    Rectangle::new(Point::new(x, y + row), Size::new(2, 1))
                        .into_styled(*fill)
                        .draw(target)
                        .ok();
                }
            }
        } else {
            Rectangle::new(Point::new(x, y), Size::new(2, bar_h as u32))
                .into_styled(*fill)
                .draw(target)
                .ok();
        }
    }

    pub fn blank(target: &mut impl DrawTarget<Color = BinaryColor>) {
        let _ = target.clear(BinaryColor::Off);
    }
}
