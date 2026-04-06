use core::fmt::Write;

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X15_BOLD};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::renderer::CharacterStyle;
use embedded_graphics::text::{Alignment, Text};
use heapless::String;

use crate::channel::{RadioState, RadioStatus};
use crate::protocol::Bandwidth;

/// RSSI ring buffer length — supports displays up to 256px wide (2px per bar).
pub const RSSI_HISTORY_LEN: usize = 128;

// Font metrics (FONT_6X10)
const CHAR_W: i32 = 6;
const FONT_H: i32 = 10;

// Mode box: 2 chars + 1px padding each side
const MODE_BOX_W: i32 = 2 * CHAR_W + 2; // 14px

// RSSI mapping range (dBm)
const RSSI_MIN: i16 = -120;
const RSSI_MAX: i16 = 0;

/// Static board identity info for display rendering.
pub struct BoardInfo<'a> {
    pub name: &'a str,
    pub version: &'a str,
    pub mac: &'a str,
}

/// Render the active dashboard (shown when radio is in RX or TX mode).
pub fn dashboard(
    target: &mut impl DrawTarget<Color = BinaryColor>,
    status: &RadioStatus,
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
    let sep1_y = header_h + 3; // 3px gap below header text (clears descenders)
    let info_y = sep1_y + 1; // 1px line, info text tight below separator
    let sep2_y = info_y + 2 * FONT_H + 3; // 3px gap below info text (clears descenders)
    let spark_top = sep2_y + 2; // 1px line + 1px gap above graph
    let spark_h = h - spark_top;
    let visible_bars = ((w / 2) as usize).min(RSSI_HISTORY_LEN);

    let _ = target.clear(BinaryColor::Off);
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let mut buf: String<32> = String::new();

    // ── Mode box (left column, 2 rows tall) ─────────────────────────
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);
    let mut inv_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::Off);
    inv_style.set_background_color(Some(BinaryColor::On));

    match status.state {
        RadioState::Receiving => {
            // "RX" inverted on row 0
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
        RadioState::Transmitting => {
            // "TX" inverted on row 1
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
        RadioState::Idle => {} // Not shown on active screen
    }

    // Vertical separator between mode box and title box
    Line::new(Point::new(MODE_BOX_W, 0), Point::new(MODE_BOX_W, sep1_y))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(target)
        .ok();

    // ── Title box (right of mode box) ───────────────────────────────
    let title_center_x = title_x + title_w / 2;

    // Row 0: "DongLoRa v{version}"
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

    // Row 1: radio settings "{freq}/{bw}/{sf}/{cr}"
    if let Some(cfg) = status.config {
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

    // ── First separator line ────────────────────────────────────────
    Line::new(Point::new(0, sep1_y), Point::new(w - 1, sep1_y))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(target)
        .ok();

    // ── Info rows (centered) ────────────────────────────────────────
    let center_x = w / 2;

    // Row 2: packet counters
    buf.clear();
    let _ = write!(
        buf,
        "RX:{} TX:{}",
        compact_count(status.rx_count),
        compact_count(status.tx_count)
    );
    Text::with_alignment(
        &buf,
        Point::new(center_x, info_y + FONT_H - 1),
        style,
        Alignment::Center,
    )
    .draw(target)
    .ok();

    // Row 3: RSSI + SNR
    buf.clear();
    match (status.last_rssi, status.last_snr) {
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

    // ── Second separator line ───────────────────────────────────────
    Line::new(Point::new(0, sep2_y), Point::new(w - 1, sep2_y))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(target)
        .ok();

    // ── Sparkline bar graph ─────────────────────────────────────────
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

/// Render the splash/waiting screen (shown when idle or no config).
pub fn splash(target: &mut impl DrawTarget<Color = BinaryColor>, board: &BoardInfo) {
    let bb = target.bounding_box();
    let w = bb.size.width as i32;

    let _ = target.clear(BinaryColor::Off);
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let title_style = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);
    let mut buf: String<32> = String::new();

    let center_x = w / 2;

    // Row 1: "DongLoRa" bold left, version small right
    Text::new("DongLoRa", Point::new(4, 15), title_style)
        .draw(target)
        .ok();
    buf.clear();
    let _ = write!(buf, "v{}", board.version);
    Text::with_alignment(&buf, Point::new(w - 2, 15), style, Alignment::Right)
        .draw(target)
        .ok();

    // Row 2: board name
    Text::with_alignment(
        board.name,
        Point::new(center_x, 28),
        style,
        Alignment::Center,
    )
    .draw(target)
    .ok();

    // Row 3: MAC address
    Text::with_alignment(
        board.mac,
        Point::new(center_x, 41),
        style,
        Alignment::Center,
    )
    .draw(target)
    .ok();

    // Row 4: status
    Text::with_alignment(
        "Waiting for host...",
        Point::new(center_x, 54),
        style,
        Alignment::Center,
    )
    .draw(target)
    .ok();
}

/// Format a u32 count compactly for display.
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

/// Render the RSSI history as a bar-chart sparkline.
///
/// The current (uncommitted) slot is rendered at the rightmost position
/// so that incoming packets appear on the graph immediately.
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

    // Draw committed history
    for i in 0..hist_slots {
        let idx = if count <= RSSI_HISTORY_LEN {
            // Buffer hasn't wrapped: slots occupy indices 0..committed-1.
            // Skip oldest entries that don't fit on screen.
            let effective_bars = if live { visible_bars - 1 } else { visible_bars };
            i + committed.saturating_sub(effective_bars)
        } else {
            let start = count - RSSI_HISTORY_LEN;
            let skip =
                RSSI_HISTORY_LEN.saturating_sub(if live { visible_bars - 1 } else { visible_bars });
            (start + skip + i) % RSSI_HISTORY_LEN
        };
        let is_tx = tx_history[idx];
        let rssi = history[idx];

        if let Some(bar_h) = bar_height(rssi, is_tx, spark_h) {
            let x = (visible_bars - total + i) as i32 * 2;
            draw_bar(target, x, bar_h, is_tx, spark_top, spark_h, &fill);
        }
    }

    // Draw live (current) bar at the rightmost position
    if live {
        if let Some(bar_h) = bar_height(current_rssi, current_tx, spark_h) {
            let x = (visible_bars - 1) as i32 * 2;
            draw_bar(target, x, bar_h, current_tx, spark_top, spark_h, &fill);
        }
    }
}

/// Compute the pixel height for a sparkline bar, or None to skip.
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

/// Draw a single sparkline bar (solid for RX, dotted for TX).
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

/// Clear the display (display-off state).
pub fn blank(target: &mut impl DrawTarget<Color = BinaryColor>) {
    let _ = target.clear(BinaryColor::Off);
}
