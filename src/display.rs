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
use crate::protocol::LoRaConfig;

const BOARD_NAME: &str = Board::NAME;
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Number of slots in the dashboard's RSSI/TX sparkline ring buffer.
pub const RSSI_HISTORY_LEN: usize = 128;

// Mono splash board-name font selection. The mono OLEDs we drive are
// 128 px wide; the primary font (FONT_6X10, 6 px advance) fits up to 21
// ASCII chars across, the fallback (FONT_5X8, 5 px advance) fits up to
// 25. If even the fallback can't fit on a mono OLED, fail at compile
// time so the board porter shortens NAME rather than silently
// overflowing the splash. Color renderers have their own size budget
// and simply use the same NAME with their own font choice.
const SPLASH_SCREEN_W_PX: u32 = 128;
const SPLASH_FONT_PRIMARY_W_PX: u32 = 6;
const SPLASH_FONT_FALLBACK_W_PX: u32 = 5;
pub(crate) const BOARD_NAME_FITS_PRIMARY_FONT: bool =
    (BOARD_NAME.len() as u32) * SPLASH_FONT_PRIMARY_W_PX <= SPLASH_SCREEN_W_PX;
const _: () = assert!(
    (BOARD_NAME.len() as u32) * SPLASH_FONT_FALLBACK_W_PX <= SPLASH_SCREEN_W_PX,
    "Board::NAME is too long for the 128 px mono splash screen even with the FONT_5X8 fallback \
     (max 25 ASCII chars). Shorten the NAME constant in src/board/<board>.rs.",
);

/// Sentinel: no packet received in this slot. Below SX1262 sensitivity
/// floor (-120 dBm), so it cannot be confused with a real RSSI value.
const NO_SIGNAL: i16 = -121;

/// Hand-authored 64 × 64 pixel-perfect QR bitmap for the splash
/// "learn more" screen. 1 bit/pixel, MSB-first, row-major. Bit `1` is
/// "light" (OLED pixel on), bit `0` is "dark" (OLED pixel off), so when
/// drawn with `BinaryColor::On` = lit, the QR appears with the
/// traditional dark-on-light look including a proper quiet zone. The
/// color renderer remaps bit 1 → white, bit 0 → black for the same
/// visual contrast on a black-background TFT.
pub(crate) const QR_WIDTH_PX: u32 = 64;
pub(crate) const QR_BITMAP: [u8; 512] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x07, 0x9F, 0xFE, 0x60, 0x00, 0x7F, 0xFE,
    0x00, 0x07, 0x9F, 0xFE, 0x60, 0x00, 0x7F, 0xFE, 0x7F, 0xE6, 0x18, 0x19, 0xE7, 0xFE, 0x7F, 0xFE,
    0x7F, 0xE6, 0x18, 0x19, 0xE7, 0xFE, 0x7F, 0xFE, 0x60, 0x67, 0x81, 0xE6, 0x66, 0x06, 0x7F, 0xFE,
    0x60, 0x67, 0x81, 0xE6, 0x66, 0x06, 0x7F, 0xFE, 0x60, 0x66, 0x19, 0x87, 0xE6, 0x06, 0x7F, 0xFE,
    0x60, 0x66, 0x19, 0x87, 0xE6, 0x06, 0x7F, 0xFE, 0x60, 0x67, 0xE6, 0x1F, 0xE6, 0x06, 0x7F, 0xFE,
    0x60, 0x67, 0xE6, 0x1F, 0xE6, 0x06, 0x7F, 0xFE, 0x7F, 0xE6, 0x67, 0x80, 0x67, 0xFE, 0x7F, 0xFE,
    0x7F, 0xE6, 0x67, 0x80, 0x67, 0xFE, 0x7F, 0xFE, 0x00, 0x06, 0x66, 0x66, 0x60, 0x00, 0x7F, 0xFE,
    0x00, 0x06, 0x66, 0x66, 0x60, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x61, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFE, 0x61, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x60, 0x06, 0x01, 0x99, 0x99, 0xFF, 0xFE,
    0x00, 0x60, 0x06, 0x01, 0x99, 0x99, 0xFF, 0xFE, 0x01, 0x98, 0x1F, 0x9E, 0x79, 0xF9, 0xFF, 0xFE,
    0x01, 0x98, 0x1F, 0x9E, 0x79, 0xF9, 0xFF, 0xFF, 0xE0, 0x66, 0x18, 0x61, 0x9F, 0x98, 0x7F, 0xFF,
    0xE0, 0x66, 0x18, 0x61, 0x9F, 0x98, 0x7F, 0xFF, 0x98, 0x1E, 0x01, 0xE7, 0x99, 0xFE, 0x7F, 0xFF,
    0x98, 0x1E, 0x01, 0xE7, 0x99, 0xFE, 0x7F, 0xFF, 0xE1, 0xE6, 0x19, 0xF8, 0x06, 0x60, 0x7F, 0xFF,
    0xE1, 0xE6, 0x19, 0xF8, 0x06, 0x60, 0x7F, 0xFE, 0x19, 0xF8, 0x06, 0x1E, 0x79, 0x99, 0xFF, 0xFE,
    0x19, 0xF8, 0x06, 0x1E, 0x79, 0x99, 0xFF, 0xFE, 0x7E, 0x01, 0xFF, 0x80, 0x18, 0x18, 0x7F, 0xFE,
    0x7E, 0x01, 0xFF, 0x80, 0x18, 0x18, 0x7F, 0xFE, 0x7E, 0x19, 0x9E, 0x1E, 0x78, 0x7E, 0x7F, 0xFE,
    0x7E, 0x19, 0x9E, 0x1E, 0x78, 0x7E, 0x7F, 0xFE, 0x60, 0x06, 0x66, 0x66, 0x00, 0x67, 0xFF, 0xFE,
    0x60, 0x06, 0x66, 0x66, 0x00, 0x67, 0xFF, 0xFF, 0xFF, 0xFE, 0x67, 0xF8, 0x7E, 0x1F, 0xFF, 0xFF,
    0xFF, 0xFE, 0x67, 0xF8, 0x7E, 0x1F, 0xFF, 0xFE, 0x00, 0x06, 0x00, 0x7E, 0x66, 0x60, 0x7F, 0xFE,
    0x00, 0x06, 0x00, 0x7E, 0x66, 0x60, 0x7F, 0xFE, 0x7F, 0xE7, 0x99, 0xE6, 0x7E, 0x1E, 0x7F, 0xFE,
    0x7F, 0xE7, 0x99, 0xE6, 0x7E, 0x1E, 0x7F, 0xFE, 0x60, 0x66, 0x61, 0xF8, 0x00, 0x60, 0x7F, 0xFE,
    0x60, 0x66, 0x61, 0xF8, 0x00, 0x60, 0x7F, 0xFE, 0x60, 0x66, 0x06, 0x01, 0x86, 0x00, 0x7F, 0xFE,
    0x60, 0x66, 0x06, 0x01, 0x86, 0x00, 0x7F, 0xFE, 0x60, 0x66, 0x67, 0x86, 0x7F, 0x86, 0x7F, 0xFE,
    0x60, 0x66, 0x67, 0x86, 0x7F, 0x86, 0x7F, 0xFE, 0x7F, 0xE6, 0x66, 0x18, 0x00, 0x1E, 0x7F, 0xFE,
    0x7F, 0xE6, 0x66, 0x18, 0x00, 0x1E, 0x7F, 0xFE, 0x00, 0x06, 0x66, 0x67, 0x98, 0x00, 0x7F, 0xFE,
    0x00, 0x06, 0x66, 0x67, 0x98, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
];

/// Which dashboard badge is in view (drives the RX/TX indicator only).
#[derive(Debug, Clone, Copy)]
pub enum Badge {
    Rx,
    Tx,
}

// ── Display content types (shared across all renderers) ───────────────
//
// `BoardInfo` and `DashboardCtx` describe *what* to draw on the splash
// and dashboard screens. Each board's `BoardDisplay` impl decides *how*
// to render that content for its native pixel format and resolution.

/// Static-ish board identity rendered on the splash + dashboard.
pub struct BoardInfo<'a> {
    pub name: &'a str,
    pub version: &'a str,
    pub mac: &'a str,
}

/// Live dashboard data at the moment of a render call.
pub struct DashboardCtx<'a> {
    pub badge: Badge,
    pub config: Option<LoRaConfig>,
    pub rx_count: u32,
    pub tx_count: u32,
    pub last_rssi: Option<i16>,
    pub last_snr: Option<i16>,
    pub rssi_history: &'a [i16; RSSI_HISTORY_LEN],
    pub tx_history: &'a [bool; RSSI_HISTORY_LEN],
    pub rssi_count: usize,
    pub current_slot_rssi: i16,
    pub current_slot_tx: bool,
    pub board: &'a BoardInfo<'a>,
}

// ── BoardDisplay trait ────────────────────────────────────────────────
//
// One logical display surface, four content states (splash×2, dashboard,
// blank), plus brightness control and a flush hook. Each board's display
// driver implements this directly — color depth, resolution, dimming
// strategy, and any framebuffer flushing live behind the trait.

/// Single trait every board's display driver implements. The display
/// task talks to the trait; the per-driver impl handles native format.
///
/// `present()` is named distinctly from any inherent `flush()` method on
/// concrete display types so method resolution doesn't get ambiguous.
pub trait BoardDisplay {
    fn present(&mut self) -> impl core::future::Future<Output = ()>;
    fn set_bright(&mut self) -> impl core::future::Future<Output = ()>;
    fn set_dim(&mut self) -> impl core::future::Future<Output = ()>;

    fn render_splash_learn_more(&mut self);
    fn render_splash_info(&mut self, info: &BoardInfo<'_>);
    fn render_dashboard(&mut self, ctx: &DashboardCtx<'_>);
    fn render_blank(&mut self);
}

// Color renderer module — used by `crate::driver::st7789::St7789Color` on
// the Heltec T114 to draw native 240×135 Rgb565 dashboards/splash screens.
// Gated to the T114 feature so other boards don't pay for the imports.
#[cfg(feature = "heltec_mesh_node_t114")]
pub mod render_color;

// Shared QR-code blit, generic over color. Used by both the mono and
// color splash screens to render the bitmap with proper quiet zones.
mod qr;

// ── Mono-OLED adapters ────────────────────────────────────────────────
//
// SH1106 + SSD1306Async both render `BinaryColor` content into a 128×64
// framebuffer flushed via I2C. Their `BoardDisplay` impls all delegate
// to `mod render` — the existing pixel-perfect mono renderer.

#[cfg(any(
    feature = "wio_tracker_l1",
    feature = "elecrow_thinknode_m2",
    feature = "lilygo_tbeam_supreme",
    feature = "rak_wisblock_4631"
))]
impl<I> BoardDisplay for crate::driver::sh1106::Sh1106<I>
where
    I: embedded_hal_async::i2c::I2c,
{
    async fn present(&mut self) {
        let _ = crate::driver::sh1106::Sh1106::flush(self).await;
    }
    async fn set_bright(&mut self) {
        crate::driver::DisplayBrightness::set_bright(self).await;
    }
    async fn set_dim(&mut self) {
        crate::driver::DisplayBrightness::set_dim(self).await;
    }
    fn render_splash_learn_more(&mut self) {
        render::learn_more(self);
    }
    fn render_splash_info(&mut self, info: &BoardInfo<'_>) {
        render::info(self, info);
    }
    fn render_dashboard(&mut self, ctx: &DashboardCtx<'_>) {
        render::dashboard(self, ctx);
    }
    fn render_blank(&mut self) {
        render::blank(self);
    }
}

impl<DI, SIZE> BoardDisplay
    for ssd1306::Ssd1306Async<DI, SIZE, ssd1306::mode::BufferedGraphicsModeAsync<SIZE>>
where
    DI: display_interface::AsyncWriteOnlyDataCommand,
    SIZE: ssd1306::size::DisplaySizeAsync,
{
    async fn present(&mut self) {
        let _ = ssd1306::Ssd1306Async::flush(self).await;
    }
    async fn set_bright(&mut self) {
        crate::driver::DisplayBrightness::set_bright(self).await;
    }
    async fn set_dim(&mut self) {
        crate::driver::DisplayBrightness::set_dim(self).await;
    }
    fn render_splash_learn_more(&mut self) {
        render::learn_more(self);
    }
    fn render_splash_info(&mut self, info: &BoardInfo<'_>) {
        render::info(self, info);
    }
    fn render_dashboard(&mut self, ctx: &DashboardCtx<'_>) {
        render::dashboard(self, ctx);
    }
    fn render_blank(&mut self) {
        render::blank(self);
    }
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
    ConfigChanged(LoRaConfig),
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
    config: Option<LoRaConfig>,
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
        display.render_splash_info(&info);
        display.present().await;
    }

    async fn flush_learn_more(&mut self) {
        let display = &mut self.peripherals.display;
        display.render_splash_learn_more();
        display.present().await;
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
        let ctx = DashboardCtx {
            badge,
            config: self.run.config,
            rx_count: self.run.rx_count,
            tx_count: self.run.tx_count,
            last_rssi: self.run.last_rssi,
            last_snr: self.run.last_snr,
            rssi_history: &self.run.rssi_history,
            tx_history: &self.run.tx_history,
            rssi_count: self.run.rssi_count,
            current_slot_rssi: self.run.current_slot_rssi,
            current_slot_tx: self.run.current_slot_tx,
            board: &info,
        };
        display.render_dashboard(&ctx);
        display.present().await;
    }

    async fn flush_blank(&mut self) {
        let display = &mut self.peripherals.display;
        display.render_blank();
        display.present().await;
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

    async fn oled_bright(&mut self) {
        self.peripherals.display.set_bright().await;
    }

    async fn oled_dim(&mut self) {
        self.peripherals.display.set_dim().await;
    }
}

// ── Statechart ─────────────────────────────────────────────────────

statechart! {
    Display {
        context: DisplayContext;
        events: DisplayEvent;
        // Auto-trace — backend selected by `hsmc/trace-*` cargo feature.
        trace;
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
            // state, turn off any blink-in-progress, and land in LearnMore
            // with a fresh 5 s timer. We target `LearnMore` (not `Splash`)
            // because under hsmc 0.2 up-transition semantics, targeting a
            // state that's already on the active path would only unwind
            // below it — we'd park at `Splash` with no active child and
            // the alternation timer would never restart.
            on(CmdReset) => reset_run_state, turn_led_off, LearnMore;
            // Config can arrive before the dashboard is up; cache it anywhere.
            on(ConfigChanged(cfg: LoRaConfig)) => save_new_config;

            // Splash is bright for the first 10 s of (re-)entry, then the
            // action-only `on(after 10s)` handler drops the OLED to dim.
            // LearnMore ↔ Info keep alternating underneath — their own
            // timers are independent of the 10 s window.
            state Splash {
                entry: set_bright;
                default(LearnMore);
                on(EnteredRx) => Listening;
                on(EnteredTx) => Transmitting;
                on(after Duration::from_secs(10)) => set_dim;

                state LearnMore {
                    entry: paint_learn_more;
                    on(after Duration::from_secs(5)) => Info;
                }

                state Info {
                    entry: paint_info;
                    on(after Duration::from_secs(5)) => LearnMore;
                }
            }

            // Dashboard brightness pattern: each mode-leaf holds a `Bright`
            // substate. `entry: set_bright` owns the backlight going on;
            // `set_dim` is fired **as an action** by the 3 s timeout
            // handler, immediately before the up-transition that parks
            // the machine at the leaf. The symmetric `exit: set_dim`
            // would also fire on mode-change leaf-crossings (e.g.
            // ListeningBright → TransmittingBright via EnteredTx), and
            // the subsequent default descent would fire `set_bright`
            // again — producing a visible dim/bright flash on every
            // TX↔RX swap. Anchoring dim to the *timer* handler instead
            // means leaf-crossings go bright→bright with no intermediate
            // dim action. CmdOff/CmdReset exit paths don't fire set_dim
            // either, which is fine — the display is blanked or walked
            // back to Splash, which re-asserts its own brightness.
            state Dashboard {
                default(Listening);
                on(RadioIdle) => Splash;

                state Listening {
                    entry: paint_listening;
                    default(ListeningBright);
                    on(EnteredTx) => Transmitting;
                    on(every Duration::from_millis(1000)) => advance_sparkline, paint_listening;
                    // Split from the transition below because hsmc rejects
                    // payload bindings on transition-bearing handlers.
                    on(PacketRx { rssi: i16, snr: Option<i16> }) => record_rx_packet, blink_rx_led;
                    on(PacketRx) => ListeningBright;
                    on(PacketTx) => record_tx_packet, blink_tx_led;
                    on(PacketTx) => ListeningBright;

                    state ListeningBright {
                        entry: set_bright;
                        on(after Duration::from_secs(3)) => set_dim, Listening;
                    }
                }

                state Transmitting {
                    entry: paint_transmitting;
                    default(TransmittingBright);
                    on(EnteredRx) => Listening;
                    on(every Duration::from_millis(1000)) => advance_sparkline, paint_transmitting;
                    on(PacketRx { rssi: i16, snr: Option<i16> }) => record_rx_packet, blink_rx_led;
                    on(PacketRx) => TransmittingBright;
                    on(PacketTx) => record_tx_packet, blink_tx_led;
                    on(PacketTx) => TransmittingBright;

                    state TransmittingBright {
                        entry: set_bright;
                        on(after Duration::from_secs(3)) => set_dim, Transmitting;
                    }
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

    async fn save_new_config(&mut self, cfg: LoRaConfig) {
        self.run.config = Some(cfg);
    }

    async fn advance_sparkline(&mut self) {
        self.run.advance_slot();
    }

    async fn set_bright(&mut self) {
        self.oled_bright().await;
    }

    async fn set_dim(&mut self) {
        self.oled_dim().await;
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

    let Some(display) = board::create_display(parts).await else {
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

    use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X10, FONT_9X15_BOLD};
    use embedded_graphics::mono_font::MonoTextStyle;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::prelude::*;
    use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
    use embedded_graphics::text::renderer::CharacterStyle;
    use embedded_graphics::text::{Alignment, Text};
    use heapless::String;

    use super::{Badge, BoardInfo, DashboardCtx, QR_WIDTH_PX, RSSI_HISTORY_LEN};
    use crate::protocol::LoRaBandwidth;

    const CHAR_W: i32 = 6;
    const FONT_H: i32 = 10;
    const MODE_BOX_W: i32 = 2 * CHAR_W + 2;

    const RSSI_MIN: i16 = -120;
    const RSSI_MAX: i16 = 0;


    pub fn dashboard(target: &mut impl DrawTarget<Color = BinaryColor>, ctx: &DashboardCtx<'_>) {
        let badge = ctx.badge;
        let config = ctx.config;
        let rx_count = ctx.rx_count;
        let tx_count = ctx.tx_count;
        let last_rssi = ctx.last_rssi;
        let last_snr = ctx.last_snr;
        let rssi_history = ctx.rssi_history;
        let tx_history = ctx.tx_history;
        let rssi_count = ctx.rssi_count;
        let current_slot_rssi = ctx.current_slot_rssi;
        let current_slot_tx = ctx.current_slot_tx;
        let board = ctx.board;
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
                LoRaBandwidth::Khz7 => "7.8",
                LoRaBandwidth::Khz10 => "10.4",
                LoRaBandwidth::Khz15 => "15.6",
                LoRaBandwidth::Khz20 => "20.8",
                LoRaBandwidth::Khz31 => "31.2",
                LoRaBandwidth::Khz41 => "41.7",
                LoRaBandwidth::Khz62 => "62.5",
                LoRaBandwidth::Khz125 => "125",
                LoRaBandwidth::Khz250 => "250",
                LoRaBandwidth::Khz500 => "500",
                LoRaBandwidth::Khz200 => "200",
                LoRaBandwidth::Khz400 => "400",
                LoRaBandwidth::Khz800 => "800",
                LoRaBandwidth::Khz1600 => "1600",
            };
            // Show only the CR denominator; the "4/" numerator is implied for
            // every LoRa coding rate and just clutters the line.
            let cr_str = match cfg.cr {
                crate::protocol::LoRaCodingRate::Cr4_5 => "5",
                crate::protocol::LoRaCodingRate::Cr4_6 => "6",
                crate::protocol::LoRaCodingRate::Cr4_7 => "7",
                crate::protocol::LoRaCodingRate::Cr4_8 => "8",
            };
            let _ = write!(
                buf,
                "{}.{:03}/{}/{}/{}",
                freq_mhz, freq_khz, bw_str, cfg.sf, cr_str
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

        // Board-name font picked at compile time by
        // BOARD_NAME_FITS_PRIMARY_FONT (see top of display.rs). If the
        // name fits FONT_6X10 we use it; otherwise we fall back to
        // FONT_5X8. The const_assert at the BOARD_NAME definition
        // rejects names that overflow even the fallback.
        if super::BOARD_NAME_FITS_PRIMARY_FONT {
            Text::with_alignment(
                board.name,
                Point::new(center_x, 28),
                style,
                Alignment::Center,
            )
            .draw(target)
            .ok();
        } else {
            let small_style = MonoTextStyle::new(&FONT_5X8, BinaryColor::On);
            Text::with_alignment(
                board.name,
                Point::new(center_x, 28),
                small_style,
                Alignment::Center,
            )
            .draw(target)
            .ok();
        }

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
    /// right (centered within its quiet zone via `super::qr::draw`),
    /// centered text column on the left inviting the user to visit the URL.
    pub fn learn_more(target: &mut impl DrawTarget<Color = BinaryColor>) {
        use embedded_graphics::mono_font::ascii::{FONT_4X6, FONT_7X14_BOLD};

        let bb = target.bounding_box();
        let w = bb.size.width as i32;

        let _ = target.clear(BinaryColor::Off);

        // ── QR code: rightmost 64 × 64 region, centered cells ───────
        let qr_x = w - QR_WIDTH_PX as i32;
        super::qr::draw(
            target,
            Point::new(qr_x, 0),
            BinaryColor::On,
            BinaryColor::Off,
        );

        // ── Text column (everything left of the QR) ─────────────────
        let col_w = qr_x;
        let col_center = col_w / 2;

        let title_style = MonoTextStyle::new(&FONT_7X14_BOLD, BinaryColor::On);
        // FONT_4X6 (4 px advance) on the URL line so "donglora.com"
        // (12 chars × 4 = 48 px) fits the 64 px left column with ~8 px
        // breathing room either side, instead of FONT_5X8's 60 px which
        // visually touches both edges.
        let text_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);

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
