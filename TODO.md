# Firmware TODO List

## `lora-phy` fix shipped locally; upstream PR pending submission

Branch: [`swaits/lora-rs @ fix/sx126x-busy-race`](https://github.com/swaits/lora-rs/tree/fix/sx126x-busy-race).
PR not yet opened. Firmware consumes the fix via `[patch.crates-io]`
pointing at `../../lora-rs/lora-phy`; drop the patch line + pre-release
selector once upstream releases.

Hardware-verified on Wio Tracker L1: 20/20 clean-boot TX after the fix
(vs ~50% before). Adds a pre-command BUSY check before each
SpiInterface read/write op, keeping the existing post-command wait in
place. Belt-and-suspenders bracketing: pre-wait per spec, post-wait
closes the BUSY-assertion latency window (T_SW ≤ 600 ns) that a
single level-poll can miss.

### PR body

```
Title: phy: pre-command BUSY check per Semtech datasheets

## Problem

While bringing up a downstream project (donglora), I hit a race where TX commands would occasionally fail. Tracked it down to lora-phy not following the BUSY-handshake algorithm Semtech recommends: the host should wait on BUSY *before* issuing each command, not after.

Details: `SpiInterface::{write, write_with_payload, read,
read_with_status}` polls BUSY *after* each SPI op.

On fast MCUs (nRF52840 at 64 MHz, release build) this is racy — `embedded-hal-async`'s `wait_for_low` returns immediately if the pin is already low, and the chip takes up to T_SW ns (SX1261/2 §8.3, Fig. 8-3) to assert BUSY after NSS rises. The host polls inside that pre-assertion window, sees a stale LOW, fires the next command while the chip is still busy, and the chip silently drops it. First `lora.tx()` after a clean boot on Seeed Wio Tracker L1 hangs ~50% of the time.

## Authority

- [SX1261/2 datasheet, DS.SX1261-2.W.APP Rev. 1.2](https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf) §8.3.1
  "BUSY Control Line": *"it is essential to wait for the BUSY line to
  go low before sending an SPI command."*
- [LR1121 User Manual, UM.LR1121.W.APP Rev. 1.2](https://files.waveshare.com/wiki/Core1121/UserManual_LR1121_v1_2.pdf) §3
  "Host-Controller Interface": *"it is necessary to check the status
  of BUSY prior to sending a command."* (Applies to the LR11xx family
  — see also [LR1110 DS Rev. 1.1](https://download.mikroe.com/documents/datasheets/LR1110_datasheet.pdf) §1.2.5.)
- [RadioLib](https://github.com/jgromes/RadioLib/blob/cffd5af/src/Module.cpp)
  `Module::SPIwriteStream` / `SPIreadStream` call `waitForBusy()` at
  the start of every transaction.
- The [SX1276/77/78/79 datasheet](https://cdn.sparkfun.com/assets/7/7/3/2/2/SX1276_Datasheet.pdf)
  Table 2 exposes no BUSY pin, so
  `GenericSx127xInterfaceVariant::wait_on_busy` stays a no-op and the
  change is behaviorally identical for SX127x.

## Fix

Bracket each `SpiInterface` / `Lr1110SpiInterface` read/write op with
both a pre- and post-command BUSY wait. The pre-wait is the new
addition required by the spec; the existing post-wait is preserved as
the belt that closes the BUSY-assertion latency window.

Why pre-wait alone isn't enough: `embedded-hal-async`'s
`wait_for_low` returns immediately when the pin is already low. The
chip takes up to T_SW ns to assert BUSY after NSS rises, so a single
polling chance — at any side of the SPI op — can see a stale LOW and
race the next command into a still-busy chip. The repro is
back-to-back writes inside `sx126x::do_cad` (SetCADParams immediately
followed by SetCAD): pre-wait alone races, SetCAD is dropped, CADDone
never fires, `lora.cad()` hangs forever. Keeping the post-wait gives a
second polling chance after enough time has elapsed for BUSY to be
reliably observable.

The `is_sleep_command` flag now gates both waits. One callsite flip:
`sx126x::ensure_ready`'s wake-from-sleep GetStatus pulse passes
`is_sleep_command=true` so the pre-wait doesn't hang on the
BUSY-stuck-HIGH chip-asleep state. The next SPI op's pre-wait absorbs
the wake-completion latency.

No public API signature changes.

## Test

- `cargo clippy -p lora-phy --features defmt-03 -- -D warnings` clean.
- Hardware: Seeed Wio Tracker L1 (nRF52840 + SX1262), 20 consecutive
  clean-boot TX cycles → 20/20 (vs ~50% before).
```

---

## Drop `[patch.crates-io]` + upgrade esp-hal / esp-rtos + add `esp_app_desc!()` + bump espflash to v4

Single coordinated change that clears our last remaining dep-graph debt
on the ESP boards. All four pieces are coupled, so they land together.

### Why

Today `Cargo.toml` overrides `esp-hal`, `esp-rtos`, `esp-backtrace`, and
`esp-println` with a git pin to `esp-rs/esp-hal#main`. The note in that
block explains the historical reason:

> REVISIT: esp-rtos 0.2.0 on crates.io pins embassy-executor ^0.9. The
> esp-rs/esp-hal main branch already supports 0.10. Remove these
> patches once esp-rtos publishes a release with embassy-executor 0.10
> support.

**That condition is now met.** `esp-rtos 0.3.0` is published on crates.io
and depends on `embassy-executor ^0.10` and `esp-hal ~1.1.0-rc.0` — exactly
what our patches were pulling unofficially. Keeping the patches is no
longer free: it blocks `espflash` v4, blocks `esp-bootloader-esp-idf`, and
ties reproducible builds to whatever commit happens to be on
`esp-rs/esp-hal#main`.

The immediate trigger is `espflash` v4's required
`esp_bootloader_esp_idf::esp_app_desc!()` check. `save-image` refuses any
ESP binary that lacks the IDF app descriptor. The crate that provides the
macro (`esp-bootloader-esp-idf`) uses the same `esp-rom-sys` native
library as `esp-hal` — two copies of the same `links = "esp_rom_sys"` is
a hard cargo error, so `esp-bootloader-esp-idf` must come from the _same
source_ as `esp-hal`. Easiest way to guarantee that: both from crates.io,
no patches.

### What changes

1. **`Cargo.toml`** — delete the entire `[patch.crates-io]` block and the
   preceding `REVISIT:` comment. Bump the four esp-rs deps to published
   versions:
   - `esp-hal = "1.0"` → `"1.1.0-rc.0"` (pre-release selector — see Gotchas)
   - `esp-rtos = "0.2"` → `"0.3"`
   - `esp-backtrace = "0.18"` → whatever matches esp-rtos 0.3 (check its deps)
   - `esp-println = "0.16"` → same
     Add a new optional dep + feature wiring for
     `esp-bootloader-esp-idf = "0.5"` with `defmt` feature, plus per-board
     chip feature (`esp-bootloader-esp-idf/esp32s3` for the S3 boards,
     `esp-bootloader-esp-idf/esp32` for LilyGo T-Beam classic).

2. **`src/main.rs`** — inside the ESP-family branch of the existing
   `cfg_if!` block (around line 51), invoke the macro at module scope:

   ```rust
   esp_bootloader_esp_idf::esp_app_desc!();
   ```

   Zero runtime cost. Emits ~256 bytes in `.rodata_desc` with app
   metadata (name, version, build date) that the ESP-IDF 2nd-stage
   bootloader + `espflash save-image` look for.

3. **`mise.toml`** — `"cargo:espflash" = "3"` → `"4"`. Drop the deferral
   comment on that line.

4. **Possible API drift fixes** — esp-hal's pre-release versions
   occasionally rename or move items. Most likely touch points:
   - `src/board/esp32.rs`, `src/board/esp32s3.rs`
   - Any board file in `src/board/heltec_*.rs`, `lilygo_*.rs`,
     `elecrow_*.rs` that calls into esp-hal / esp-rtos init
     Run `just check-all` after steps 1–3 and patch up whatever surfaces.
     Don't guess — read the actual compiler errors.

### How (step by step)

1. `jj new -m "chore(esp): drop esp-hal patches, bump to published versions, add esp_app_desc!()"`
2. Edit `Cargo.toml` — delete `[patch.crates-io]` + `REVISIT:` comment;
   bump the four versions; add `esp-bootloader-esp-idf` dep + feature
   wiring for each ESP board feature block.
3. Edit `src/main.rs` — add `esp_bootloader_esp_idf::esp_app_desc!();`
   to the ESP branch of the top-level `cfg_if!`.
4. Edit `mise.toml` — espflash `"3"` → `"4"`.
5. `mise install` to pull espflash 4.x.
6. `cargo update` to let the resolver pick a consistent tree with the
   new requirements.
7. `just check-all` — expect some compilation errors from API drift on
   ESP boards. Fix them one at a time; consult the official examples
   in `esp-rs/esp-hal` if in doubt.
8. `just build heltec_v3` (or any ESP board you have) — verify
   `espflash save-image` now succeeds end-to-end producing a valid
   `.bin`.
9. Flash to real hardware if available. **The RAK4631 and Wio Tracker
   L1 (nRF52) boards should be untouched.**

### Verification

- `just check-all` clean on all 9 boards.
- `just test` (host-side protocol tests) still green.
- For each ESP board that has been physically validated before
  (Heltec V3, LilyGo T-Beam units): flash + confirm radio TX works
  end-to-end via ai-bot or a known peer.
- RAK4631 + Wio Tracker L1 unchanged; re-check via `just check` on
  both, but no re-flash needed (they don't go through the ESP code
  path).

### Gotchas

- **Pre-release version selector**: `esp-hal = "1.1.0-rc.0"` by itself
  does NOT match pre-release versions in cargo's default semver rules.
  Use an exact pre-release like `"1.1.0-rc.0"`, `"~1.1.0-rc.0"` (only
  `1.1.0-rc.0`, `1.1.0-rc.1`, … with any patch), or `">=1.1.0-rc.0"`.
  `"1.1"` will NOT resolve to a pre-release.
- **Two copies of `esp-rom-sys`**: the root cause of the first failed
  attempt at this task. If cargo errors with _"Only one package in the
  dependency graph may specify the same links value"_ for
  `esp_rom_sys`, it means `esp-hal` and `esp-bootloader-esp-idf` are
  being pulled from different sources (one crates.io, one git). Either
  both from crates.io (this task's plan) or both from the same git
  commit.
- **`esp-bootloader-esp-idf` chip features**: the crate needs exactly
  one of `esp32 / esp32s3 / esp32c3 / esp32c6 / …` to be enabled.
  Without it, the build panics at its `build.rs`: _"Expected exactly
  one of the following features to be enabled: esp32, esp32c2, ..."_.
  Wire per-board the same way we wire `esp-hal/esp32s3` etc.
- **Probe-less repro**: `mise install` after bumping espflash to "4"
  only installs mise's copy. If the user's global
  `~/.cargo/bin/espflash` is stale/different, it still wins on PATH.
  The justfile `set shell := ["mise", "exec", "--", "sh", "-c"]`
  prepends mise's paths but does NOT reorder `~/.cargo/bin` ahead of
  them. Either `cargo uninstall espflash` globally, or change the
  justfile recipes to call `$(mise which espflash)` explicitly.

### Why not now

Tried it alongside the RAK4631 bring-up in April 2026 and bounced off
the `esp-rom-sys` two-copies problem. Backed out the attempt and left
espflash pinned at v3 + the patches in place so the bring-up commit
stayed atomic and shippable. This task is what's needed to unblock v4
properly, but it deserves its own focused session — not as
"while-I'm-here" scope on a board bring-up.
