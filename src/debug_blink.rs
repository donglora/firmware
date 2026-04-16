//! LED-based diagnostic blinker (opt-in via the `debug-checkpoint` cargo
//! feature).
//!
//! On boards whose UART is also the host protocol (heltec_v3_uart, any
//! UART-only variant), defmt output would corrupt the COBS-framed host
//! protocol. We can't use `defmt::info!` for live debugging on those
//! boards while the host is connected. Instead we expose a single global
//! atomic checkpoint; code sets it at interesting points in the boot /
//! dispatch flow, and a dedicated task continuously flashes the board
//! LED N times, pauses, and repeats — so a human can count the flashes
//! and report "it got to N."
//!
//! Blink pattern:
//! * Each flash: 400 ms ON, 400 ms OFF.
//! * After N flashes: 2000 ms pause (LED held OFF).
//! * Repeat indefinitely with the current `CHECKPOINT` value.
//!
//! Usage:
//! ```ignore
//! #[cfg(feature = "debug-checkpoint")]
//! crate::debug_blink::set(3);  // "we reached checkpoint 3"
//! ```
//!
//! The task replaces `display_task` when the `debug-checkpoint` feature is
//! enabled — it takes the board LED and drives it exclusively.
//!
//! # Current checkpoint map
//!
//! Defined by the `crate::radio` module. Updated here any time the
//! numbering changes there.
//!
//! | Flashes | Meaning                                                |
//! |--------:|--------------------------------------------------------|
//! |   1     | `radio_task` entered (pre-`LoRa::new`)                  |
//! |   2     | `LoRa::new` succeeded                                   |
//! |   3     | Machine constructed; about to enter `run()`             |
//! |   4     | In `Idle` state                                         |
//! |   5     | In `Receiving` state                                    |
//! |   6     | TX phase 1: entered `perform_tx` (engages latch)        |
//! |   7     | TX phase 2: in CAD                                      |
//! |   8     | TX phase 3: in `prepare_for_tx`                         |
//! |   9     | TX phase 4: awaiting `tx()`                             |
//! |  10     | TX phase 5: `do_tx` returned OK                         |
//! |  11     | TX phase 6: `respond_tx_done` handler entered           |
//! |  12     | TX phase 7: `responses.send(TxDone)` returned           |
//! |  13     | TX phase 8: `host_task` received the response from chan |
//! |  14     | TX phase 9: `host_task` wrote response bytes to UART — TERMINAL SUCCESS |
//! |  15     | TX errored somewhere — TERMINAL ERROR                   |
//! |  16     | Boot error: `LoRa::new` failed                          |
//! |  17     | Boot error: `start_rx` failed inside `start_rx_hw`      |
//!
//! # Latching
//!
//! * Latch engages **only** on `set(6)` (entry to `perform_tx`).
//! * While latched, only TX-phase values (6–15) can update `CHECKPOINT`,
//!   and only if strictly deeper than the current value.
//! * Terminals (14 success, 15 error) freeze the latch until reboot.
//! * `set(13)` and `set(14)` called from `host_task` are **no-ops before
//!   the latch engages** — this prevents unrelated SetConfig/Ok responses
//!   from polluting the display before any TX has happened.
//!
//! # Diagnostic logic
//!
//! If ai-bot reports a timeout but the LED shows:
//! * **10** → `do_tx` ok, but `TransmitDone` event never dispatched.
//! * **11** → handler entered, but `responses.send(...)` hung.
//! * **12** → send pushed to channel, but `host_task` never picked it up.
//! * **13** → `host_task` picked it up, but `tx.write_all` to UART hung.
//! * **14** → full end-to-end firmware success. Hang is on the host side
//!   of the UART (mux or wire).
//! * **15** → TX errored somewhere along the way.
//!
//! # Building with the feature
//!
//! The `just` build/flash recipes pass a fixed feature list and don't
//! surface an `extra-features` argument. To build a debug image:
//!
//! ```sh
//! cd firmware
//! . ~/export-esp.sh   # if building for xtensa (esp32s3)
//! cargo +esp build --release \
//!     --target xtensa-esp32s3-none-elf \
//!     --features heltec_v3_uart,debug-checkpoint \
//!     -Zbuild-std=core,alloc
//! cp target/xtensa-esp32s3-none-elf/release/donglora \
//!    builds/donglora-heltec_v3_uart-v$(sed -n 's/^version = "\(.*\)"/\1/p' \
//!    Cargo.toml).elf
//! espflash flash -p /dev/ttyUSB0 builds/donglora-heltec_v3_uart-v0.4.0.elf
//! ```
//!
//! The normal `just flash heltec_v3_uart` command builds without the
//! feature, so the debug code has zero footprint on release firmware.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_executor::task;
use embassy_time::{Duration, Timer};

use crate::board::{LedDriver, RgbLed};

/// Current diagnostic checkpoint — read by the blinker task. Zero means
/// "no checkpoint set yet."
///
/// Write policy (see [`set`]): once a TX-phase value (6, 11, 12, 13) is
/// stored, the latch engages and:
/// * subsequent TX-phase writes update the value only if strictly deeper,
/// * non-TX-phase writes (including the 7/10 terminal markers and the
///   steady-state 4/5 post-TX writes) are **ignored** so the deepest phase
///   reached on the FIRST TX attempt persists.
///
/// Rationale: a stuck-then-recovered TX attempt would normally flicker
/// through the display too quickly to count. Latching preserves the
/// evidence until reboot.
pub static CHECKPOINT: AtomicU8 = AtomicU8::new(0);

/// Has the TX-phase latch engaged yet? Written by [`set`], checked by
/// [`set`]. Not intended for external inspection.
static HAS_LATCHED: AtomicBool = AtomicBool::new(false);

#[inline]
fn is_host_phase(n: u8) -> bool {
    // Values set only from `host_task` — meaningless before a TX has
    // started, so we refuse to overwrite with them when the latch is
    // not yet engaged.
    n == 13 || n == 14
}

/// Ordered depth of each TX-related checkpoint. Returns `0` for values
/// that are NOT TX phases (e.g. boot / steady-state / error indicators).
///
/// Value 18 ("next_command received CmdTransmit") is intentionally the
/// SHALLOWEST TX phase — it fires before `perform_tx` entry (value 6)
/// but is numerically higher because it was added last. The latch uses
/// `phase_depth` rather than raw numeric comparison so 18 can engage
/// the latch at depth 1 and later TX phases (6..=15) deepen over it.
#[inline]
fn phase_depth(n: u8) -> u8 {
    match n {
        18 => 1,  // pre-TX: CmdTransmit just read from commands channel
        6 => 2,   // perform_tx entered
        7 => 3,   // in CAD
        8 => 4,   // in prepare_for_tx
        9 => 5,   // in tx().await
        10 => 6,  // do_tx returned Ok
        11 => 7,  // respond_tx_done entered
        12 => 8,  // respond_tx_done: responses.send returned
        13 => 9,  // host_task received response from channel
        14 => 10, // host_task wrote response to UART (terminal success)
        15 => 11, // TX errored (terminal error)
        _ => 0,   // not a TX phase
    }
}

#[inline]
fn is_tx_phase(n: u8) -> bool {
    phase_depth(n) > 0
}

#[inline]
fn is_tx_terminal(n: u8) -> bool {
    n == 14 || n == 15
}

/// Record that we reached checkpoint `n`. A no-op in terms of visible
/// behavior unless the blinker task is spawned; the store itself is very
/// cheap so leaving these calls in non-debug builds is harmless.
///
/// Latch semantics:
/// * `set(18)` (CmdTransmit received by `next_command`) or `set(6)`
///   (entry to `perform_tx`) engage the latch.
/// * While latched, only strictly-deeper TX phases (by `phase_depth`)
///   update the checkpoint.
/// * Terminal values (14 success, 15 error) freeze the latch until
///   reboot.
/// * Host-phase values (13, 14) called from `host_task` are ignored
///   before the latch engages, so unrelated `Response::Ok` writes from
///   SetConfig don't pollute the LED reading before any TX.
#[inline]
pub fn set(n: u8) {
    if HAS_LATCHED.load(Ordering::Relaxed) {
        let cur = CHECKPOINT.load(Ordering::Relaxed);
        if is_tx_terminal(cur) {
            return;
        }
        if phase_depth(n) > phase_depth(cur) {
            CHECKPOINT.store(n, Ordering::Relaxed);
        }
        return;
    }

    // Latch not yet engaged.
    if is_host_phase(n) {
        // `host_task` is draining the response channel before any TX —
        // these values would be misleading. Skip them.
        return;
    }

    CHECKPOINT.store(n, Ordering::Relaxed);
    if is_tx_phase(n) {
        HAS_LATCHED.store(true, Ordering::Relaxed);
    }
}

/// Dedicated task that continuously flashes the LED to show the current
/// checkpoint. Owns the board's LED driver exclusively — on boards where
/// the display task would normally own the LED for RX/TX blips, spawn
/// *either* this task *or* `display_task`, not both.
#[task]
pub async fn debug_blink_task(mut led: LedDriver) {
    // Start with the LED off so the first flash is unambiguous.
    led.set_rgb(0, 0, 0).await;

    loop {
        let n = CHECKPOINT.load(Ordering::Relaxed);
        if n == 0 {
            // No checkpoint yet — wait and re-check.
            Timer::after(Duration::from_millis(200)).await;
            continue;
        }
        for _ in 0..n {
            led.set_rgb(255, 255, 255).await;
            Timer::after(Duration::from_millis(400)).await;
            led.set_rgb(0, 0, 0).await;
            Timer::after(Duration::from_millis(400)).await;
        }
        // Long pause to separate cycles so the human can count.
        Timer::after(Duration::from_millis(2000)).await;
    }
}
