//! Pure-math helpers for Listen-Before-Talk (CAD) retry timing.
//!
//! The CAD retry loop in `radio::do_tx` scales its backoff to the active
//! LoRa configuration: per-retry sleep is one time-on-air of an assumed
//! peer packet, total retry budget is capped at `CAD_BUDGET_US`. This
//! module owns the ToA math (Semtech AN1200.13) and the retry-count
//! derivation, split out so it can be unit-tested host-side without the
//! radio module's hardware dependencies.

use crate::protocol::{LoRaCodingRate, LoRaConfig, LoRaHeaderMode};

/// Total LBT budget: the maximum time the CAD retry loop is allowed to
/// tie up the radio before returning `ChannelBusy`.
pub const CAD_BUDGET_US: u32 = 2_500_000;
/// Upper bound on retries, independent of budget — caps the work on
/// very fast configs (SF7/BW500 would otherwise get 60+ retries).
pub const CAD_HARD_CAP: u8 = 10;
/// Assumed peer-packet payload (bytes) used to size the per-retry sleep.
/// 80 is a representative median for typical mesh traffic. Peers sending
/// larger packets may be partially under-waited (accepted trade-off —
/// host-level retry handles collisions).
#[allow(dead_code)] // consumed by `radio.rs`, which is gated out of test builds
pub const CAD_ASSUMED_PEER_PAYLOAD_B: u16 = 80;
/// Jitter applied to each sleep, in tenths of a percent (±20 % = 200).
pub const CAD_JITTER_TENTHS_PCT: u32 = 200;

/// ToA formula expects CR ∈ {1..4} for 4/5..4/8 (each coded symbol carries
/// `CR+4` bits). The protocol enum starts at 0, so map explicitly — using
/// `as u32` alone would underestimate airtime by one code bit everywhere.
pub fn cr_numeric(cr: LoRaCodingRate) -> u32 {
    match cr {
        LoRaCodingRate::Cr4_5 => 1,
        LoRaCodingRate::Cr4_6 => 2,
        LoRaCodingRate::Cr4_7 => 3,
        LoRaCodingRate::Cr4_8 => 4,
    }
}

/// Worst-case LoRa time-on-air in microseconds (Semtech AN1200.13).
///
/// Uses `N_preamble + 4.25` quarter-symbols for preamble duration; SX126x
/// uses 6.25 at SF5/SF6, so the result is low by 2·T_sym at those SFs
/// (sub-millisecond — negligible at the LBT-budget scale).
pub fn toa_us(cfg: &LoRaConfig, payload_len: u16) -> u32 {
    let sf = cfg.sf as u64;
    let bw_hz = cfg.bw.as_hz() as u64;
    // Symbol time in µs: (2^SF / BW_Hz) * 1e6. u64 avoids overflow at
    // SF12/BW7 where T_sym ≈ 524 ms.
    let t_sym_us = (1u64 << sf) * 1_000_000 / bw_hz.max(1);
    let de: i64 = if t_sym_us > 16_000 { 1 } else { 0 };
    let crc: i64 = if cfg.payload_crc { 1 } else { 0 };
    let h: i64 = if matches!(cfg.header_mode, LoRaHeaderMode::Implicit) {
        1
    } else {
        0
    };
    let pl = payload_len as i64;
    let sf_i = sf as i64;

    // Payload symbol count (signed intermediates — `num` goes negative
    // for small PL at SF12 with H=1, CRC=0).
    let num = 8 * pl - 4 * sf_i + 28 + 16 * crc - 20 * h;
    let den = 4 * (sf_i - 2 * de);
    let payload_sym_ceil = if num <= 0 || den <= 0 {
        0i64
    } else {
        (num + den - 1) / den
    };
    let cr_num = cr_numeric(cfg.cr) as i64;
    let n_payload = 8 + (payload_sym_ceil * (cr_num + 4)).max(0);
    let t_payload_us = n_payload as u64 * t_sym_us;

    // Preamble duration: (N + 4.25) * T_sym = (4·N + 17) * T_sym / 4.
    let t_preamble_us = (cfg.preamble_len as u64 * 4 + 17) * t_sym_us / 4;

    (t_preamble_us + t_payload_us).min(u32::MAX as u64) as u32
}

/// Derive the CAD retry count and per-retry sleep from the cached
/// `cad_retry_delay_us`. Retries form: N CADs separated by N-1 sleeps.
/// When one assumed-packet wait exceeds the total budget, `retries = 1`
/// — a single CAD, bail on busy, no sleep.
pub fn cad_retries(cad_retry_delay_us: u32) -> (u8, u32) {
    if cad_retry_delay_us == 0 {
        // Unconfigured should never reach this path; fall back to a
        // single CAD with no sleep rather than divide-by-zero.
        return (1, 0);
    }
    let n_sleeps = CAD_BUDGET_US / cad_retry_delay_us;
    let retries = (n_sleeps + 1).clamp(1, CAD_HARD_CAP as u32) as u8;
    (retries, cad_retry_delay_us)
}

/// Apply ±`CAD_JITTER_TENTHS_PCT`/10 % jitter around `base_us`.
/// `entropy` is a raw 32-bit source; caller provides it so this stays
/// pure (radio task passes `Instant::now().as_ticks() as u32`).
pub fn jittered_sleep_us(base_us: u32, entropy: u32) -> u32 {
    let mixed = entropy.wrapping_mul(0x9E37_79B9);
    let span = 2 * CAD_JITTER_TENTHS_PCT + 1; // e.g. 401 for ±20 %
    let r = mixed % span; // 0..=2·CAD_JITTER_TENTHS_PCT
    // factor_thousandths ∈ [1000 − CAD_JITTER_TENTHS_PCT, 1000 + CAD_JITTER_TENTHS_PCT]
    let factor_thousandths = 1000 - CAD_JITTER_TENTHS_PCT + r;
    ((base_us as u64 * factor_thousandths as u64) / 1000) as u32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{LoRaBandwidth, LoRaCodingRate, LoRaConfig, LoRaHeaderMode};

    fn cfg(sf: u8, bw: LoRaBandwidth, cr: LoRaCodingRate, preamble: u16, implicit: bool, crc: bool) -> LoRaConfig {
        LoRaConfig {
            freq_hz: 915_000_000,
            sf,
            bw,
            cr,
            preamble_len: preamble,
            sync_word: 0x1424,
            tx_power_dbm: 0,
            header_mode: if implicit {
                LoRaHeaderMode::Implicit
            } else {
                LoRaHeaderMode::Explicit
            },
            payload_crc: crc,
            iq_invert: false,
        }
    }

    /// Helper: assert `actual` is within `tolerance_pct`% of `expected_us`.
    fn approx(actual_us: u32, expected_us: u32, tolerance_pct: u32) {
        let diff = actual_us.abs_diff(expected_us);
        let tol = expected_us * tolerance_pct / 100;
        assert!(
            diff <= tol,
            "actual {} us not within {}% of expected {} us (diff {}, tol {})",
            actual_us,
            tolerance_pct,
            expected_us,
            diff,
            tol
        );
    }

    // Reference values sanity-checked against Semtech's LoRa calculator
    // and AN1200.13.

    #[test]
    fn toa_sf7_bw125_cr45_pl10_explicit_crc() {
        let c = cfg(7, LoRaBandwidth::Khz125, LoRaCodingRate::Cr4_5, 8, false, true);
        // Hand-computed reference from AN1200.13:
        //   T_sym = 128/125 kHz = 1.024 ms
        //   T_preamble = (8 + 4.25) · 1.024 = 12.544 ms
        //   n_payload = 8 + ceil((80 − 28 + 28 + 16) / 28) · 5 = 8 + 4·5 = 28
        //   T_payload = 28 · 1.024 = 28.672 ms
        //   Total ≈ 41.22 ms
        approx(toa_us(&c, 10), 41_216, 2);
    }

    #[test]
    fn toa_sf12_bw125_cr45_pl10_explicit_crc() {
        let c = cfg(12, LoRaBandwidth::Khz125, LoRaCodingRate::Cr4_5, 8, false, true);
        // Reference: ~991 ms for PL=10 at SF12/BW125/CR4/5 (LDRO on).
        approx(toa_us(&c, 10), 991_000, 2);
    }

    #[test]
    fn toa_sf10_bw125_cr45_pl255_max() {
        let c = cfg(10, LoRaBandwidth::Khz125, LoRaCodingRate::Cr4_5, 8, false, true);
        // Reference: ~2.34 s for PL=255 at SF10/BW125/CR4/5.
        approx(toa_us(&c, 255), 2_335_000, 2);
    }

    #[test]
    fn toa_ldro_activates_at_sf11_and_sf12_bw125() {
        // T_sym > 16 ms → DE=1. SF11/BW125: T_sym = 16.384 ms. SF10/BW125: 8.192 ms.
        let sf10 = cfg(10, LoRaBandwidth::Khz125, LoRaCodingRate::Cr4_5, 8, false, true);
        let sf11 = cfg(11, LoRaBandwidth::Khz125, LoRaCodingRate::Cr4_5, 8, false, true);
        let sf12 = cfg(12, LoRaBandwidth::Khz125, LoRaCodingRate::Cr4_5, 8, false, true);
        // Crude check: each step roughly doubles airtime (LDRO doesn't
        // break the doubling trend — it just shifts constants).
        let t10 = toa_us(&sf10, 80);
        let t11 = toa_us(&sf11, 80);
        let t12 = toa_us(&sf12, 80);
        assert!(t11 > t10 * 15 / 10, "SF11 should be > 1.5x SF10, got {} vs {}", t11, t10);
        assert!(t12 > t11 * 15 / 10, "SF12 should be > 1.5x SF11, got {} vs {}", t12, t11);
    }

    #[test]
    fn toa_assumed_peer_80b_matches_plan_table() {
        let mk = |sf, bw| cfg(sf, bw, LoRaCodingRate::Cr4_5, 8, false, true);
        // Plan table values (approximate, within 5%).
        approx(toa_us(&mk(7, LoRaBandwidth::Khz500), 80), 36_000, 5);
        approx(toa_us(&mk(7, LoRaBandwidth::Khz250), 80), 72_000, 5);
        approx(toa_us(&mk(7, LoRaBandwidth::Khz125), 80), 144_000, 5);
        approx(toa_us(&mk(8, LoRaBandwidth::Khz125), 80), 257_000, 5);
        approx(toa_us(&mk(9, LoRaBandwidth::Khz125), 80), 452_000, 5);
        approx(toa_us(&mk(10, LoRaBandwidth::Khz125), 80), 862_000, 5);
        approx(toa_us(&mk(11, LoRaBandwidth::Khz125), 80), 1_806_000, 5);
        approx(toa_us(&mk(12, LoRaBandwidth::Khz125), 80), 3_285_000, 5);
    }

    #[test]
    fn toa_no_underflow_at_small_pl_sf12_implicit_no_crc() {
        // num = 8·PL − 4·SF + 28 + 16·CRC − 20·H goes negative here; the
        // i64 + max(0) guard must kick in without panicking or wrapping.
        let c = cfg(12, LoRaBandwidth::Khz125, LoRaCodingRate::Cr4_5, 8, true, false);
        let t = toa_us(&c, 1);
        // Only 8 payload symbols survive (the formula floors to 0 extra);
        // should be preamble + 8·T_sym ≈ 401 + 262 = 663 ms roughly.
        approx(t, 663_000, 5);
    }

    // Retry-count derivation table from the plan.

    #[test]
    fn retries_sf7_bw500_hits_hard_cap() {
        let (n, s) = cad_retries(36_000);
        assert_eq!(n, 10);
        assert_eq!(s, 36_000);
    }

    #[test]
    fn retries_sf10_bw125() {
        // T_assumed ≈ 862 ms; budget 2.5 s ⇒ 2 sleeps ⇒ 3 retries.
        let (n, _) = cad_retries(862_000);
        assert_eq!(n, 3);
    }

    #[test]
    fn retries_sf11_bw125() {
        // T_assumed ≈ 1.81 s; budget 2.5 s ⇒ 1 sleep ⇒ 2 retries.
        let (n, _) = cad_retries(1_806_000);
        assert_eq!(n, 2);
    }

    #[test]
    fn retries_sf12_bw125_degraded() {
        // T_assumed ≈ 3.29 s > 2.5 s budget ⇒ 0 sleeps ⇒ 1 retry (bail on busy).
        let (n, _) = cad_retries(3_285_000);
        assert_eq!(n, 1);
    }

    #[test]
    fn retries_zero_delay_falls_back_to_single_cad() {
        // Unconfigured safety: avoid divide-by-zero.
        let (n, s) = cad_retries(0);
        assert_eq!(n, 1);
        assert_eq!(s, 0);
    }

    // Jitter sanity.

    #[test]
    fn jitter_stays_within_pm_20_pct() {
        let base = 1_000_000u32;
        // Exhaustive over the low byte — enough to cover the `% span` range.
        let mut saw_low = false;
        let mut saw_high = false;
        for e in 0u32..=u16::MAX as u32 {
            let j = jittered_sleep_us(base, e);
            // ±20% ⇒ [800_000, 1_200_000].
            assert!(
                (800_000..=1_200_000).contains(&j),
                "jitter {} us out of ±20% bounds for entropy {}",
                j,
                e
            );
            if j <= 850_000 {
                saw_low = true;
            }
            if j >= 1_150_000 {
                saw_high = true;
            }
        }
        assert!(saw_low && saw_high, "jitter didn't span the expected range");
    }

    #[test]
    fn jitter_zero_base_is_zero() {
        for e in [0u32, 1, 42, 0xDEAD_BEEF] {
            assert_eq!(jittered_sleep_us(0, e), 0);
        }
    }
}
