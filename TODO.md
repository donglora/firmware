# Firmware TODO List

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
