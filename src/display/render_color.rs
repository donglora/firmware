//! Native 240×135 Rgb565 renderer for the Heltec T114 ST7789 panel.
//!
//! Mirrors the content shape of the mono OLED renderer in `display::render`
//! (same `BoardInfo` / `DashboardCtx` inputs) but lays things out for the
//! T114's bigger color screen with native font sizes and colored badges.
//! Each board's `BoardDisplay` impl picks the renderer that matches its
//! native pixel format — no per-pixel color quantization, no scaling,
//! no aliasing.

use core::fmt::Write;

use embedded_graphics::mono_font::ascii::{
    FONT_6X10, FONT_8X13, FONT_8X13_BOLD, FONT_9X15_BOLD, FONT_10X20,
};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::renderer::CharacterStyle;
use embedded_graphics::text::{Alignment, Text};
use heapless::String;

use super::{qr, Badge, BoardInfo, DashboardCtx, QR_WIDTH_PX, RSSI_HISTORY_LEN};
use crate::protocol::LoRaBandwidth;

// ── Panel dimensions ──────────────────────────────────────────────────

const W: i32 = 240;
const H: i32 = 135;

// ── Palette ───────────────────────────────────────────────────────────

const BG: Rgb565 = Rgb565::BLACK;
const FG: Rgb565 = Rgb565::WHITE;
const DIM: Rgb565 = Rgb565::new(12, 24, 12); // muted gray
const ACCENT: Rgb565 = Rgb565::new(8, 30, 31); // bright cyan
const RX_COLOR: Rgb565 = Rgb565::new(8, 60, 8); // bright green
const TX_COLOR: Rgb565 = Rgb565::new(31, 16, 8); // amber-red
const SPARK_COLOR: Rgb565 = Rgb565::new(20, 50, 24); // soft green

// ── Sparkline ────────────────────────────────────────────────────────

const RSSI_MIN: i16 = -120;
const RSSI_MAX: i16 = 0;
/// Sentinel: no packet received in this slot. Mirrors `display::NO_SIGNAL`.
const NO_SIGNAL: i16 = -121;

// ── Public render entry points ────────────────────────────────────────

pub fn blank<D: DrawTarget<Color = Rgb565>>(target: &mut D) {
    let _ = target.clear(BG);
}

pub fn splash_learn_more<D: DrawTarget<Color = Rgb565>>(target: &mut D) {
    let _ = target.clear(BG);

    // QR pinned to the right side, vertically centered. 64×64 fits with
    // ~36 px headroom top/bottom on the 135-tall panel.
    let qr_w = QR_WIDTH_PX as i32;
    let qr_x = W - qr_w - 8;
    let qr_y = (H - qr_w) / 2;
    qr::draw(target, Point::new(qr_x, qr_y), FG, BG);

    // Text column to the left of the QR.
    let col_w = qr_x - 8;
    let col_center = col_w / 2 + 4;

    let title_style = MonoTextStyle::new(&FONT_10X20, ACCENT);
    let url_style = MonoTextStyle::new(&FONT_8X13, FG);
    let caption_style = MonoTextStyle::new(&FONT_6X10, DIM);

    let _ = Text::with_alignment(
        "DongLoRa",
        Point::new(col_center, 45),
        title_style,
        Alignment::Center,
    )
    .draw(target);

    let _ = Text::with_alignment(
        "donglora.com",
        Point::new(col_center, 73),
        url_style,
        Alignment::Center,
    )
    .draw(target);

    let _ = Text::with_alignment(
        "Scan for docs",
        Point::new(col_center, 95),
        caption_style,
        Alignment::Center,
    )
    .draw(target);
}

pub fn splash_info<D: DrawTarget<Color = Rgb565>>(target: &mut D, info: &BoardInfo<'_>) {
    let _ = target.clear(BG);

    let title_style = MonoTextStyle::new(&FONT_10X20, ACCENT);
    let board_style = MonoTextStyle::new(&FONT_9X15_BOLD, FG);
    let mac_style = MonoTextStyle::new(&FONT_8X13, FG);
    let small_style = MonoTextStyle::new(&FONT_6X10, DIM);

    // Title at top.
    let _ = Text::with_alignment(
        "DongLoRa",
        Point::new(W / 2, 22),
        title_style,
        Alignment::Center,
    )
    .draw(target);

    // Board name.
    let _ = Text::with_alignment(
        info.name,
        Point::new(W / 2, 50),
        board_style,
        Alignment::Center,
    )
    .draw(target);

    // MAC.
    let _ = Text::with_alignment(
        info.mac,
        Point::new(W / 2, 75),
        mac_style,
        Alignment::Center,
    )
    .draw(target);

    // Version.
    let mut version_buf: String<24> = String::new();
    let _ = write!(version_buf, "v{}", info.version);
    let _ = Text::with_alignment(
        &version_buf,
        Point::new(W / 2, 95),
        small_style,
        Alignment::Center,
    )
    .draw(target);

    // Waiting line.
    let _ = Text::with_alignment(
        "Waiting for host...",
        Point::new(W / 2, 120),
        small_style,
        Alignment::Center,
    )
    .draw(target);
}

pub fn dashboard<D: DrawTarget<Color = Rgb565>>(target: &mut D, ctx: &DashboardCtx<'_>) {
    let _ = target.clear(BG);

    // ── Top bar (badge + radio config) ──────────────────────────
    let top_h = 24;
    let badge_w = 38;
    let (badge_color, badge_text) = match ctx.badge {
        Badge::Rx => (RX_COLOR, "RX"),
        Badge::Tx => (TX_COLOR, "TX"),
    };

    let badge_rect = Rectangle::new(Point::zero(), Size::new(badge_w as u32, top_h as u32));
    let _ = badge_rect
        .into_styled(PrimitiveStyle::with_fill(badge_color))
        .draw(target);

    let mut badge_style = MonoTextStyle::new(&FONT_8X13_BOLD, FG);
    badge_style.set_background_color(Some(badge_color));
    let _ = Text::with_alignment(
        badge_text,
        Point::new(badge_w / 2, 16),
        badge_style,
        Alignment::Center,
    )
    .draw(target);

    // Radio config text right of the badge.
    let cfg_style = MonoTextStyle::new(&FONT_8X13, FG);
    let mut buf: String<48> = String::new();
    if let Some(cfg) = ctx.config {
        let bw_str = bw_label(cfg.bw);
        let cr_str = cr_label(cfg.cr);
        let freq_mhz = cfg.freq_hz / 1_000_000;
        let freq_khz = (cfg.freq_hz % 1_000_000) / 1_000;
        let _ = write!(
            buf,
            "{}.{:03} {} SF{} CR{}",
            freq_mhz, freq_khz, bw_str, cfg.sf, cr_str
        );
    } else {
        let _ = write!(buf, "Awaiting config");
    }
    let _ = Text::with_alignment(
        &buf,
        Point::new(W - 4, 16),
        cfg_style,
        Alignment::Right,
    )
    .draw(target);

    // Separator line under top bar.
    let _ = Line::new(Point::new(0, top_h + 1), Point::new(W - 1, top_h + 1))
        .into_styled(PrimitiveStyle::with_stroke(DIM, 1))
        .draw(target);

    // ── Bottom bar (counters + RSSI) ───────────────────────────
    let bottom_h = 30;
    let bottom_top = H - bottom_h;
    let _ = Line::new(
        Point::new(0, bottom_top - 1),
        Point::new(W - 1, bottom_top - 1),
    )
    .into_styled(PrimitiveStyle::with_stroke(DIM, 1))
    .draw(target);

    let stat_style = MonoTextStyle::new(&FONT_8X13, FG);

    let mut rxtx_buf: String<24> = String::new();
    let _ = write!(
        rxtx_buf,
        "RX {} TX {}",
        compact_count(ctx.rx_count).as_str(),
        compact_count(ctx.tx_count).as_str()
    );
    let _ = Text::with_alignment(
        &rxtx_buf,
        Point::new(4, bottom_top + 11),
        stat_style,
        Alignment::Left,
    )
    .draw(target);

    let mut rssi_buf: String<32> = String::new();
    match (ctx.last_rssi, ctx.last_snr) {
        (Some(rssi), Some(snr)) => {
            let _ = write!(rssi_buf, "{} dBm  SNR {}", rssi, snr);
        }
        (Some(rssi), None) => {
            let _ = write!(rssi_buf, "{} dBm", rssi);
        }
        _ => {
            let _ = write!(rssi_buf, "no signal");
        }
    }
    let _ = Text::with_alignment(
        &rssi_buf,
        Point::new(W - 4, bottom_top + 11),
        stat_style,
        Alignment::Right,
    )
    .draw(target);

    let secondary = MonoTextStyle::new(&FONT_6X10, DIM);
    let _ = Text::with_alignment(
        ctx.board.mac,
        Point::new(W / 2, bottom_top + 25),
        secondary,
        Alignment::Center,
    )
    .draw(target);

    // ── Sparkline ──────────────────────────────────────────────
    let spark_top = top_h + 4;
    let spark_h = (bottom_top - 2) - spark_top;
    if spark_h > 0 {
        // 240 wide / 2 px per bar = 120 visible bars (subset of the
        // 128-slot ring). Matches the mono renderer's
        // `visible_bars = w / 2` heuristic.
        let visible_bars = ((W / 2) as usize).min(RSSI_HISTORY_LEN);
        rssi_sparkline(
            target,
            ctx.rssi_history,
            ctx.tx_history,
            ctx.rssi_count,
            ctx.current_slot_rssi,
            ctx.current_slot_tx,
            spark_top,
            spark_h,
            visible_bars,
        );
    }
}

// ── Helpers ───────────────────────────────────────────────────────────

fn bw_label(bw: LoRaBandwidth) -> &'static str {
    match bw {
        LoRaBandwidth::Khz7 => "7.8k",
        LoRaBandwidth::Khz10 => "10.4k",
        LoRaBandwidth::Khz15 => "15.6k",
        LoRaBandwidth::Khz20 => "20.8k",
        LoRaBandwidth::Khz31 => "31.2k",
        LoRaBandwidth::Khz41 => "41.7k",
        LoRaBandwidth::Khz62 => "62.5k",
        LoRaBandwidth::Khz125 => "125k",
        LoRaBandwidth::Khz250 => "250k",
        LoRaBandwidth::Khz500 => "500k",
        LoRaBandwidth::Khz200 => "200k",
        LoRaBandwidth::Khz400 => "400k",
        LoRaBandwidth::Khz800 => "800k",
        LoRaBandwidth::Khz1600 => "1.6M",
    }
}

fn cr_label(cr: crate::protocol::LoRaCodingRate) -> &'static str {
    match cr {
        crate::protocol::LoRaCodingRate::Cr4_5 => "5",
        crate::protocol::LoRaCodingRate::Cr4_6 => "6",
        crate::protocol::LoRaCodingRate::Cr4_7 => "7",
        crate::protocol::LoRaCodingRate::Cr4_8 => "8",
    }
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
fn rssi_sparkline<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
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
            draw_bar(target, x, bar_h, is_tx, spark_top, spark_h);
        }
    }

    if live {
        if let Some(bar_h) = bar_height(current_rssi, current_tx, spark_h) {
            let x = (visible_bars - 1) as i32 * 2;
            draw_bar(target, x, bar_h, current_tx, spark_top, spark_h);
        }
    }
}

fn bar_height(rssi: i16, is_tx: bool, spark_h: i32) -> Option<i32> {
    let h = if rssi <= RSSI_MIN || rssi == NO_SIGNAL {
        if is_tx {
            spark_h / 3
        } else {
            return None;
        }
    } else {
        let clamped = rssi.clamp(RSSI_MIN, RSSI_MAX);
        ((clamped - RSSI_MIN) as i32 * spark_h) / (RSSI_MAX - RSSI_MIN) as i32
    };
    if h == 0 { None } else { Some(h) }
}

fn draw_bar<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    x: i32,
    bar_h: i32,
    is_tx: bool,
    spark_top: i32,
    spark_h: i32,
) {
    let y = spark_top + spark_h - bar_h;
    let color = if is_tx { TX_COLOR } else { SPARK_COLOR };
    if is_tx {
        // Hatched bar — every other pixel row — mirrors the mono
        // renderer's TX visual cue.
        for row in 0..bar_h {
            if row % 2 == 0 {
                let _ = Rectangle::new(Point::new(x, y + row), Size::new(2, 1))
                    .into_styled(PrimitiveStyle::with_fill(color))
                    .draw(target);
            }
        }
    } else {
        let _ = Rectangle::new(Point::new(x, y), Size::new(2, bar_h as u32))
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(target);
    }
}
