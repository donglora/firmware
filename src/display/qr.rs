//! Shared QR-code blit for splash screens.
//!
//! The 512-byte QR bitmap in the parent module is laid out asymmetrically
//! within its 64×64 frame: rows 0-6 and 57-63 are top/bottom quiet zones,
//! but the QR cells start at col 0 (no left quiet zone) and extend to
//! col ~49, leaving ~14 px of white padding on the right plus one stray
//! black pixel at col 63. Rendering the bitmap as-is leaves the QR
//! visually shifted toward the left of its rendering area, which looks
//! wrong on any layout that doesn't tuck the bitmap flush against a
//! screen edge.
//!
//! [`draw`] re-centers the QR data: it skips the bitmap's right padding
//! (cols 50-63), fills the rendering area's left and right 7 px with
//! the foreground "quiet zone" color, and renders the actual QR cells
//! (bitmap cols 0..50) at +7 horizontal offset. Rows are rendered as-is
//! so the bitmap's existing top/bottom quiet zones land where expected.
//!
//! Generic over color so the mono renderer (`BinaryColor`) and the
//! T114's color renderer (`Rgb565`) share a single implementation —
//! they pass their own foreground (white / lit) and background (black /
//! unlit) values.

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::pixelcolor::PixelColor;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::Pixel;

use super::{QR_BITMAP, QR_WIDTH_PX};

/// Width of the actual QR data within the bitmap (cols 0..QR_DATA_W).
const QR_DATA_W: i32 = 50;
/// White padding on left + right to center the QR cells. With
/// QR_DATA_W=50 and total area=64 (= QR_WIDTH_PX), padding = 7 each side.
const PAD_X: i32 = (QR_WIDTH_PX as i32 - QR_DATA_W) / 2;

/// Render the 64×64 QR with proper quiet zones on all four sides.
///
/// `fg` is the "quiet zone / light cell" color (white on a TFT, "on"
/// on a mono OLED). `bg` is the "dark cell" color. Bitmap rows 0-6 and
/// 57-63 already encode top/bottom quiet zones in `fg`; this function
/// fills the missing left/right padding columns explicitly.
pub fn draw<D, C>(target: &mut D, origin: Point, fg: C, bg: C)
where
    D: DrawTarget<Color = C>,
    C: PixelColor,
{
    // Fill left padding (cols 0..PAD_X) with fg.
    let _ = target.fill_solid(
        &Rectangle::new(origin, Size::new(PAD_X as u32, QR_WIDTH_PX)),
        fg,
    );
    // Fill right padding (cols PAD_X+QR_DATA_W..QR_WIDTH_PX) with fg.
    let right_pad_w = QR_WIDTH_PX as i32 - PAD_X - QR_DATA_W;
    let _ = target.fill_solid(
        &Rectangle::new(
            Point::new(origin.x + PAD_X + QR_DATA_W, origin.y),
            Size::new(right_pad_w as u32, QR_WIDTH_PX),
        ),
        fg,
    );

    // Render bitmap cols 0..QR_DATA_W at +PAD_X horizontal offset.
    let bytes_per_row = (QR_WIDTH_PX / 8) as usize;
    let pixels = (0..QR_WIDTH_PX as i32).flat_map(move |y| {
        (0..QR_DATA_W).map(move |x| {
            let byte = QR_BITMAP[y as usize * bytes_per_row + (x as usize) / 8];
            let bit = (byte >> (7 - (x as u8 & 7))) & 1;
            let color = if bit == 1 { fg } else { bg };
            Pixel(Point::new(origin.x + PAD_X + x, origin.y + y), color)
        })
    });
    let _ = target.draw_iter(pixels);
}
