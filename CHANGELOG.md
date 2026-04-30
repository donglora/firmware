# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [1.5.0] - 2026-04-30

### Added

- **Heltec Mesh Node T114 board support** (nRF52840 + SX1262 + 1.14"
  240×135 ST7789 color TFT). The first color display board in the
  project, and the third nRF52840 + SX1262 board alongside the
  RAK4631 and Wio Tracker L1.
  - `VEXT_ENABLE` (P0.21) driven HIGH at boot to power the SX1262 +
    TFT rail. Without this, lora-phy init succeeds at the SPI level
    but TX_DONE never fires.
  - `VTFT_CTRL` on P0.03 (active-LOW); PWM backlight on P0.15
    (active-LOW, inverted) for smooth dim/bright transitions.
  - TFT SPI2 (SCK P1.08 / MOSI P1.09) at 8 MHz over EasyDMA, async
    `ExclusiveDevice` from `embedded-hal-bus`.
  - Linker uses the shared `ld/nrf52840-memory.x` so the S140
    SoftDevice region (0x01000–0x26000) is preserved per project
    policy.

- **`BoardDisplay` trait** (`src/display.rs`) bundles `render_*`,
  `present()`, `set_bright()`, and `set_dim()`. Replaces the previous
  `DrawTarget<Color = BinaryColor>` + `DisplayBrightness` combo as
  the `LoRaBoard::DisplayDriver` bound, so each board renders in its
  native pixel format and resolution without aliasing or
  color-quantization round-trips. Existing mono OLED drivers
  (`Sh1106`, `Ssd1306Async`) get adapter impls that delegate to the
  existing `display::render` module — no behavior change for any
  previously supported board.

- **Async ST7789 driver** (`src/driver/st7789.rs`) wrapping
  `lcd-async` (async fork of `mipidsi`) with a 64,800-byte raw-byte
  framebuffer in `.bss`. Renders via
  `lcd_async::raw_framebuf::RawFrameBuf<Rgb565, _>` and presents via
  a single async DMA blit (`panel.show_raw_data(...).await`); the
  await yields the executor for the ~65 ms SPI transfer so the radio
  task isn't starved during repaints.

- **Native 240×135 RGB565 renderer** (`src/display/render_color.rs`)
  with splash, dashboard, and sparkline parity with the mono path,
  plus a shared QR-code blit (`src/display/qr.rs`) generic over
  `PixelColor`. The blit re-centers the 64×64 bitmap (cells in cols
  0..50 with ~14 px right-side padding), so both mono and color
  splash now show the QR with 7 px quiet zones on all four sides
  (was visibly left-shifted on mono).

- **Shared content types** (`BoardInfo`, `DashboardCtx`) decouple
  *what* to draw from *how* each board renders it. `create_display()`
  now takes the board's full `DisplayParts` bundle (was just the I2C
  bus), so non-I2C displays can carry SPI + control pins through.

- **IDF app descriptor on every ESP image.** New
  `esp-bootloader-esp-idf` 0.5 dependency (chip-feature-gated like
  the rest of the `esp-*` stack) plus `esp_app_desc!()` in
  `src/main.rs`, which emits the `.flash.appdesc` section that
  `espflash` v4 requires in every ESP binary.

### Changed

- **ESP build modernized to upstream crates.io.** Removed the
  `[patch.crates-io]` block that pinned `esp-hal`, `esp-rtos`,
  `esp-backtrace`, and `esp-println` to `esp-rs/esp-hal#main`. The
  versions tracked there (`esp-hal 1.1`, `esp-rtos 0.3`,
  `esp-backtrace 0.19`, `esp-println 0.17`) have all reached
  crates.io with the `embassy-executor` 0.10 compatibility we needed,
  so the local fork pins are no longer required. `cargo:espflash`
  bumped to v4 in `mise.toml` to match.

- **`hsmc` 0.5.0 → 0.5.1.** Patch bump pulled in transparently with
  the lockfile refresh; no behavior change.

- **Documentation consolidated.** `PROTOCOL.md` removed from the
  firmware repo (it now lives canonically in the spec repo, linked
  from `README.md`). `BLE_MUX_DESIGN_DOC.md` moved to
  `docs/ble_mux_design.md` so all firmware design docs sit under
  `docs/`.

## [1.4.1] - 2026-04-29

### Fixed

- **Major LoRa reliability boost across every radio family.** Switched
  `lora-phy` from crates.io to the
  [`swaits/lora-rs#fixed`](https://github.com/swaits/lora-rs/tree/fixed)
  fork branch — a merge of three fixes awaiting upstream PR acceptance,
  pinned at `e8a52ebd` via `Cargo.lock`:
  - [#428](https://github.com/lora-rs/lora-rs/pull/428) — sx126x:
    pre-command BUSY check + standby/IRQ-clear hygiene around CAD.
  - [#431](https://github.com/lora-rs/lora-rs/pull/431) — sx127x:
    clear stale IRQ flags in `do_rx` before mode change.
  - [#432](https://github.com/lora-rs/lora-rs/pull/432) — sx127x:
    clear stale IRQ flags before reconfiguring DIO mapping in
    `set_irq_params`.

  Stability gain across the fleet: the sx126x BUSY-race fix lands on
  every nRF / RP2040 board, and the sx127x IRQ-clear fixes unblock the
  LilyGo T-Beam (classic) in particular.
- **UART host RX delivery on UART-only boards.** `RADIO_THROTTLED` was
  never cleared on UART (`heltec_v3_uart`, `elecrow_thinknode_m2`,
  `lilygo_tbeam`) — the throttle gate added in v1.3.0's chart-first
  restructure silently dropped every async RX from boot. Mirrored the
  USB pattern: clear the throttle on successful `tx.write_all`, re-arm
  on failure.
- **UART host watchdog spurious-trip under saturated bidirectional
  load.** The `select3`-based loop serialised read and write;
  `tx.write_all` (~25 ms per max frame at 115200 baud) blocked the read
  poll long enough for `last_frame_at` to age past 1000 ms, cascading a
  Reset to radio (`Unconfigured`) and display (splash) and leaving
  subsequent host TX commands timing out. Restructured `host_task` into
  two sub-loops joined in the same task: `read_loop` owns rx +
  watchdog, `write_loop` owns tx + throttle. Same executor, disjoint
  borrows on the split rx/tx halves, no Mutex needed. Also reset the
  `FrameDecoder` on watchdog fire so a host that resumes mid-frame
  resyncs cleanly on the next `0x00` sentinel (USB gets this for free
  via DTR-drop reset; UART has no DTR).
- **Splash board-name overflow on long names.** Names that don't fit
  `FONT_6X10` (21 ASCII chars on a 128 px-wide OLED) now fall back to
  `FONT_5X8` (25 chars). A `const_assert` at the `BOARD_NAME`
  definition rejects names that overflow even the fallback, so the
  board porter shortens `NAME` at compile time rather than getting
  silent overflow at runtime. Also dropped the URL-line font on the
  "learn more" screen from `FONT_5X8` to `FONT_4X6` so `donglora.com`
  sits inside the 64 px column with breathing room instead of touching
  both edges.

### Changed

- **`hsmc` 0.4 → 0.5.** `default(...)` is optional on every state in
  0.5; collapsed the `WaitingForFirstFrame` substate that existed only
  because 0.4 required `Connected` to declare a default child.
  `Connected` itself is now the resting "DTR up, no traffic yet" state,
  with `on(FrameReceived) => Active` descending into the
  inactivity-timer state on the first byte. Behavior is identical
  (entry/exit chain math unchanged); the host/usb chart has one fewer
  state and one fewer `default()`. Display and Radio charts left as-is
  — their defaults serve real grouping/lifecycle purposes, not 0.4
  artifacts.
- **ESP cross-toolchain pinned in `mise.toml`.** `[env]` now owns the
  PATH (`xtensa-esp-elf-gcc` bin) and `LIBCLANG_PATH`, so any `mise
  exec` resolves the linker without sourcing espup's stray
  `~/export-esp.sh`. New `[tasks.esp-install]` invokes espup with the
  matching crosstool version pin. `cargo:espup` pinned to `0.17.1` to
  match the `[env]` paths. Triggered by a real failure where
  `~/export-esp.sh` was moved to Trash and the ESP build broke with
  `linker xtensa-esp32s3-elf-gcc not found`. Bumping ESP versions
  remains a two-touch change: edit version dirs in BOTH
  `[tasks.esp-install]` and `[env]`.
- **ESP dependency requirements bumped** to track the upstream
  `esp-rs/esp-hal` main branch: `esp-hal 1.0 → 1.1`, `esp-rtos 0.2 →
  0.3`, `esp-backtrace 0.18 → 0.19`, `esp-println 0.16 → 0.17`. `cargo
  update` had silently invalidated the `[patch.crates-io]` entries (the
  patched versions were no longer semver-compatible with the old
  top-level requirements), dropping the patches and falling back to
  crates.io releases that pin `embassy-executor 0.9` — mismatched with
  our `0.10`. Catching this keeps the patch suite consistent until the
  planned `[patch.crates-io]`-removal pass.

## [1.4.0] - 2026-04-25

### Added

- **RAK4631 WisBlock board support** (nRF52840 + SX1262, RAK19007 base
  + RAK4631 Core + RAK1921 OLED). The second nRF52840 + SX1262 board
  alongside the Wio Tracker L1.
  - `SX126X_POWER_EN` (P1.05) driven HIGH at init — the radio is
    power-gated on this module; without this the chip has no power
    and SPI hangs.
  - HFXO + LFXO external crystals selected in `embassy_nrf::Config`.
    USB CDC-ACM needs ±500 ppm which the internal RC oscillator
    can't provide.
  - `SoftwareVbusDetect(true, true)` so USB enumerates without a
    `CLOCK_POWER` interrupt handler.
  - Green LED on P1.03 (RAK19007 Base LED1, active-HIGH) via
    `SimpleLed`.
  - Hardware-validated end-to-end: ADVERT round-trip with an external
    receiver on US915.
  - Added `debug-checkpoint` blink-map coverage at every TX-phase
    boundary in `radio.rs`, including a new checkpoint between
    `prepare_for_cad` and `cad()` so future bring-ups can distinguish
    a modem-setup hang from a DIO1-wait hang.

- **Adaptive listen-before-talk.** The CAD pre-TX retry loop scales to
  the active LoRa config instead of running a fixed 10 retries × 10 ms
  (which was roughly one max-length packet at SF7/BW500 and a no-op at
  SF12/BW125).
  - Per-retry sleep is sized to the time-on-air of an assumed-peer
    80-byte packet under the current SF/BW/CR — cached on
    `RadioContext`, recomputed on every `SET_CONFIG`.
  - Retry count = `clamp(floor(BUDGET/T)+1, 1, 10)` with `BUDGET = 2.5
    s`. Fast configs still get up to 10 retries; SF11/12 degrade to
    single-CAD-and-bail (logged at config-apply).
  - Sleeps jittered ±20% per iteration to decorrelate
    simultaneously-backing-off contenders.
  - ToA math, CR→{1..4} mapping, retry-count derivation, and jitter
    helper live in a pure `src/lbt.rs` module — host-testable via
    `DONGLORA_HOST_TEST=1 cargo test` (13 tests).
  - Validated on-device: 20 iterations of the AI Bot test flow that
    previously saw ~2 `ChannelBusy` errors now sees zero at the same
    SF/BW.

### Fixed

- **First-TX-after-boot reliability on SX1262 boards.** Picked up the
  upstream `lora-phy` fix that moves the BUSY-handshake check from
  *after* to *before* each SPI op, per Semtech SX1261/2 §8.3.1, LR1121
  UM §3, and LR1110 DS §1.2.5. The existing post-op wait is preserved
  as the belt that closes the BUSY-assertion latency window
  (T_SW ≤ 600 ns) that a single level-poll can miss. Consumed via
  `[patch.crates-io]` against `swaits/lora-rs#fix/sx126x-busy-race`
  until the upstream PR merges.
  - Hardware-verified on Seeed Wio Tracker L1: 20/20 clean-boot TX
    (vs ~50% before).
  - Renames `InterfaceVariant::wait_on_busy` → `wait_until_ready` to
    match the new semantics.
- **BUSY pin pull-down → none on nRF SX1262 boards.** SX1262's BUSY
  pin has an internal 20 kΩ pull-up (datasheet p. 53); our
  `Input::new(..., Pull::Down)` was contending with it, leaving BUSY
  at an indeterminate voltage and intermittently mis-reading the
  chip's ready state. Per `lora-rs/lora-rs#260`, switched to
  `Pull::None` on both Wio Tracker L1 and RAK4631. The RAK4631
  appeared to work before, but was affected by the same class of bug
  per the upstream issue — now deterministically correct rather than
  "happens to work".

### Changed

- **Build-system DX.** `just flash`, `just flash-probe`, and
  `just build` accept an optional comma-separated `extra_features`
  argument that is appended to the board feature, e.g. `just flash
  wio_tracker_l1 debug-checkpoint`. No more dropping to raw
  `cargo + rust-objcopy + uf2 cp` for iteration loops.
- **`just` recipes now resolve all tools through `mise`** via
  `set shell := ["mise", "exec", "--", "sh", "-c"]`. `cargo`,
  `espflash`, `rust-objcopy`, `cargo-hex-to-uf2` etc. always pick up
  the versions pinned in `mise.toml` regardless of `PATH` ordering.
- **`mise.toml` rust pins now carry ARM targets + clippy**, so a fresh
  `mise install` is sufficient bootstrapping — no manual
  `rustup target add` for `thumbv6m-none-eabi` /
  `thumbv7em-none-eabihf` and no separate clippy install.
- **New `just flash-dfu` recipe** for the Adafruit nRF52 bootloader
  (VID:PID `239a:0029`) via `adafruit-nrfutil` under `uvx`, with port
  autodetect.
- **Per-board `_info` chain in the justfile** collapsed from a long
  per-board `if/elif` ladder + per-board `:=` variables into a single
  `case` statement — one place per board, two lists (names +
  metadata).

## [1.3.1] - 2026-04-23

### Fixed

- **LilyGo T-Beam (classic) red LED now flashes on TX/RX** instead of
  dimming off. The red LED under the OLED is wired active-LOW on this
  board (anode to V+, cathode to GPIO4); v1.3.0 drove it as active-HIGH,
  so the LED sat lit at idle and briefly dimmed during activity.
  - Empirically verified on hardware against both T-Beam variants.
  - Introduces `src/driver/inverted_pin.rs::InvertedPin<P>`, a generic
    `OutputPin` wrapper that swaps `set_high`/`set_low`. Polarity is
    expressed at the pin (where the physical wiring lives), so
    `SimpleLed` stays polarity-agnostic and every other board's code
    path is byte-for-byte unchanged.
  - Classic T-Beam now constructs its LED as
    `SimpleLed(InvertedPin(led_pin))` with pin init at `Level::High`
    (dark at boot).

## [1.3.0] - 2026-04-23

### Added

- LilyGo T-Beam S3 Supreme board support (ESP32-S3 + SX1262 + SH1106
  OLED + AXP2101 PMIC, native USB-C).
  - Pin map verified against MeshCore's
    `variants/lilygo_tbeam_supreme_SX1262`.
  - New `src/driver/axp.rs` module with an AXP2101 init sequence
    (ALDO3 = 3.3 V LoRa, ALDO1 = 3.3 V OLED). The PMIC gates LoRa and
    display power — without it the SX1262 never responds on SPI.
  - PMIC lives on a dedicated I²C1 bus (SDA=42, SCL=41), separate
    from the SH1106 on I²C0 (SDA=17, SCL=18). I²C1 is used blocking
    for a one-shot init and then dropped.
  - TCXO driven at 1.6 V via DIO3 (MeshCore uses the same).
- LilyGo T-Beam (classic) board support (ESP32 + SX1276 + SSD1306
  OLED + AXP192 PMIC, CP2102 USB-UART bridge). This is the first
  board in the firmware on classic ESP32 (non-S3) and the first on
  the SX127x radio family.
  - Pin map verified against MeshCore's `variants/lilygo_tbeam_SX1276`.
  - New `src/hal/esp32.rs` (classic-ESP32 MCU primitives, peripheral
    DMA via `DMA_SPI2`) and `src/board/esp32.rs` (SX1276 radio helper
    + SSD1306 display pipe).
  - AXP192 init added to `src/driver/axp.rs`: DCDC1 = 3.3 V for the
    OLED, LDO2 = 3.3 V for the SX1276 PA_BOOST rail. Shares the
    OLED I²C bus (SDA=21, SCL=22); the I²C instance is built in
    blocking mode for PMIC init and converted to async for the
    display task.
  - SX127x support via lora-phy 3's `Sx127x<Sx1276>` +
    `GenericSx127xInterfaceVariant` (DIO0 as the sole IRQ; no
    BUSY line on SX127x). XTAL clocked (no TCXO), `tx_boost = true`
    for the PA_BOOST path.
  - `LoRaBoard` trait gains `supported_sf_bitmap()` and
    `supported_bw_bitmap()` methods with SX126x defaults
    (`0x1FE0`, `0x03FF`). T-Beam classic overrides SF to `0x1FC0`
    (SF6–SF12) — the SX1276 has no SF5.
  - `radio_chip_id()` reports `0x0011` (`RadioChipId::Sx1276`) and
    `freq_range_hz()` widens to 137 MHz–1020 MHz per the SX1276
    datasheet.

### Changed

- `Cargo.toml`: the `esp32s3` chip feature has been moved out of
  the `esp-hal` / `esp-rtos` / `esp-backtrace` / `esp-println`
  dependency lines and into each ESP board's feature list (as
  `esp-hal/esp32s3`, etc.). This lets `lilygo_tbeam` select the
  classic `esp32` chip flavour from the same dependency set.

## [1.2.1] - 2026-04-23

### Fixed

- **Radio no longer drops to Idle mid-RX-session on transient failures.**
  Extends the v1.1.0 invariant ("`SET_CONFIG` during RX auto-re-arms")
  to all failure paths that can occur while a client has RX active:
  - `Receiving.on(RxFailed) => Idle` now transitions to `ResumingRx`
    instead of `Idle`. A transient `next_packet` error re-arms the chip
    rather than ending the RX session.
  - `TransmittingFromRx.on(TransmitFailed) => Idle` now transitions to
    `ResumingRx`. TX failure during an active RX session returns
    `TX_FAILED` to the caller and resumes RX, matching the shape of the
    adjacent `ChannelBusy` handler.
  - `ResumingRx.on(StartRxFailed) => Idle` is now a bounded retry
    (5 × 25 ms backoff via a new `bump_resume_retries` action and a
    `ResumeRxRetry` / `ResumeRxExhausted` pair of internal events).
    The chip only lands in `Idle` after exhausting the retry budget —
    which means it's genuinely wedged and needs reconfiguring.

  Visible effect: the OLED dashboard no longer bounces to Splash when
  `ai-bot` (or any other client that TXes while RX is running) sends a
  packet. Previously any of the three transitions above would fire
  `RadioEvent::Idle`, which the display uses as its `Dashboard → Splash`
  trigger; the radio would land in `Idle` with no client aware enough
  to re-send `RX_START`, stranding the dashboard until the session was
  manually torn down and restarted.

- Removed `announce_rx_failed` helper action: its only purpose was
  emitting `RadioEvent::Idle` on the now-deleted `Receiving.on(RxFailed)
  => Idle` edge. No replacement needed — `ResumingRx` handles recovery
  silently.

## [1.2.0] - 2026-04-22

### Added

- **OLED auto-dim after inactivity.** The display is bright on any
  activity — splash entry, mode change into a dashboard leaf, packet
  RX/TX — and dims after a short idle window. Splash holds bright for
  10 s, then drops to dim while the LearnMore / Info carousel keeps
  alternating. The dashboard stays bright for 3 s after the last
  packet or mode change, then dims in place without changing
  Listening/Transmitting state. Cross-leaf TX↔RX hops transition
  bright→bright with no intermediate dim — `set_dim` is fired as an
  action by the 3 s timer handler rather than as a sibling `exit:`
  action, so mode-change leaf-crossings never see a flash.
- `DisplayBrightness` trait in `src/driver/mod.rs` unifies the
  two-level backlight API across the `ssd1306` crate's `Ssd1306Async`
  and the in-tree `Sh1106` driver. Contrast values centralized as
  `CONTRAST_BRIGHT` (`0xFF`) and `CONTRAST_DIM` (`0x08`) constants so
  the two levels can be tuned in one place.

### Changed

- `src/display.rs` statechart restructured:
  - `Splash` gains `entry: set_bright` plus an action-only
    `on(after 10s) => set_dim` handler so the splash bright window
    is time-scoped without disturbing the LM↔Info alternation.
  - `Dashboard` `Listening` and `Transmitting` each gain a nested
    `*Bright` substate owning the `entry: set_bright` action. Packet
    handlers moved into the leaves and target the leaf's `*Bright` —
    sibling-to-sibling within the leaf restarts the 3 s timer without
    re-entering the leaf itself (paint / 1 s sparkline timer
    preserved). The 3 s timeout handler fires `set_dim` as an action
    and up-transitions to the leaf, parking the machine at the leaf
    with no active child.
- `On.on(CmdReset)` now targets `LearnMore` directly rather than
  `Splash`. Under hsmc 0.2 up-transition semantics, targeting
  `Splash` from within `Splash.LearnMore` (USB-disconnect race path)
  would only unwind below `Splash` and park with no active child,
  leaving the LM↔Info alternation timer dead and the QR frozen on
  screen.
- `LoRaBoard::DisplayDriver` trait bound extended with
  `+ DisplayBrightness` so brightness is part of the board contract
  alongside `DrawTarget<Color = BinaryColor>`.

### Fixed

- Elecrow ThinkNode-M2 board doc corrected: the shipping revision
  carries a WCH CH340K USB-UART bridge (USB `1a86:7522`), not a
  Silicon Labs CP2102. `just flash elecrow_thinknode_m2` now matches
  the CH340K / CH340 VID:PIDs directly instead of falling through
  to espflash auto-detect.

### Dependencies

- `hsmc` bumped to `0.2` to pick up the up-transition fix (transition
  to an already-active ancestor unwinds only the subtree strictly
  below the target; no exit/entry on the target; no default descent).

## [1.1.0] - 2026-04-22

Small but high-value behavior change: `SET_CONFIG` issued while the
radio is in continuous RX no longer drops the radio (and the OLED)
back to idle. The radio now auto-re-arms RX with the new modulation
once reconfiguration completes, matching DongLoRa Protocol 1.1's
§3.5 / §6.3 amendments.

### Changed

- `src/radio.rs`: inside `state Receiving`, add `on(ConfigApplied)
  => ResumingRx` to shadow the parent `Configured` state's
  `on(ConfigApplied) => Idle` handler. The reconfigure path now
  runs `apply_set_config` → `ConfigApplied` → `ResumingRx.entry:
  try_start_rx_hw` → `StartRxOk` → `Receiving` without ever
  entering `Idle`.
- The OLED's `Dashboard → Splash` transition was previously driven
  by `RadioEvent::Idle` firing on `Configured.Idle.entry`. That
  event no longer fires during a SET_CONFIG-from-Receiving
  sequence, so the display stays on Dashboard.

### Dependencies

- `donglora-protocol` bumped to `1.1.0` (spec-only change; matches
  the new normative text the firmware now implements).

### Use-case unlock

- Scanner / time-division / dynamic-modulation workflows that
  rotate LoRa parameters mid-RX no longer need to re-issue
  `RX_START` after every `SET_CONFIG`. One call, radio continues
  to receive.

## [1.0.0] - 2026-04-22

The 1.0 release. Firmware now speaks DongLoRa Protocol v2 end to end,
depends on the freshly-published `donglora-protocol` 1.0.0 and `hsmc`
0.1.0 from crates.io, and ships GitHub Release binaries for every
supported board.

### Breaking

- **DongLoRa Protocol v2 wire format** (wire-incompatible with 0.x
  clients and firmware). Every `type_id`, tag-correlation rule, and
  field layout follows `PROTOCOL.md` v1.0.
- **Host framing replaced.** `src/host/framing.rs` now drives a
  streaming `FrameDecoder` from `donglora-protocol` and emits
  `(type_id, tag, payload)` frames; the 0.x COBS-per-response shim is
  gone.
- **Radio task reshape.** `src/radio.rs` emits v2 `TX_DONE` /
  `SetConfigResult` / `RxPayload` / `RxOrigin` shapes directly; no 0.x
  translation layer remains.

### Changed

- `hsmc` switched from path-dep on the local workspace to
  `version = "0.1.0"` from crates.io.
- `donglora-protocol` switched from path-dep on `../protocol` to
  `version = "1.0.0"` from crates.io; firmware builds are now
  reproducible from a clean checkout with no sibling repo required.
- `PROTOCOL.md` updated to the DongLoRa Protocol v2 normative spec,
  kept byte-for-byte in sync with the `donglora-protocol` crate's
  copy.

### Infrastructure

- Cargo.toml `repository` + `homepage` now point at the canonical
  `https://github.com/donglora/firmware` (the `swaits/donglora`
  repo is archived).

## [0.5.0] - 2026-04-16

### Added

- BLE soft-mesh multiplexer design document (`BLE_MUX_DESIGN_DOC.md`) —
  specification for turning a dongle from a 1:1 USB device into a shared
  near-field radio gateway for multiple BLE-attached phones. Three-phase
  rollout (v1 MVP, v2 hardening, v3 multi-dongle mesh) with regulatory
  duty-cycle and TX-access-control enforcement from v1. Targets nRF52840
  first; ESP32-S3 reserved for v2.
- `debug-checkpoint` cargo feature with a dedicated `debug_blink` task and
  a latching atomic checkpoint — LED-pattern diagnostic for UART-only
  boards where defmt output would corrupt the host protocol. Zero
  footprint when the feature is off.

### Changed

- Radio task rewritten as an `hsmc` statechart (`Unconfigured` /
  `Configured.{Idle, StartingRx, Receiving, ResumingRx,
  Transmitting.{FromIdle, FromRx}}`). The main loop is now a single
  `machine.run().await`; command reading, packet RX, and TX are
  state-scoped `during:` activities. Post-TX RX re-arm is modeled as
  state identity (`TransmittingFromRx` → `ResumingRx`) instead of a
  `was_receiving` flag.
- Display task rewritten as an `hsmc` statechart (`Off` /
  `On.{Splash.{LearnMore, Info}, Dashboard.{Listening, Transmitting}}`).
  The 5 s splash alternation and 1 s sparkline tick use `on(after …)` /
  `on(every …)` transitions — no external timer task.
- USB host task now carries a small `hsmc` statechart (`Disconnected` /
  `Connected`) whose entry/exit actions handle display wake, radio
  `StopRx`, and display reset — replacing the open-coded `was_connected`
  edge detector.
- Radio → Display channel switched from a `RadioStatus` watch to a typed
  `RadioEvent` queue. The radio emits discrete `EnteredRx` / `EnteredTx`
  / `Idle` / `PacketRx` / `PacketTx` / `ConfigChanged` events; downstream
  consumers observe transitions directly rather than diffing snapshots.
- Wire protocol types and COBS framing moved out of firmware into the
  shared `donglora-protocol` crate (path dep on `../protocol`). Firmware
  `src/protocol.rs` is now a re-export shim preserving every call site;
  `src/host/framing.rs` keeps only the embassy-specific `route_command`.

### Fixed

- USB disconnect recovery: on a read error the DTR-edge-detect branch
  now runs unconditionally (was early-continuing in some paths, starving
  the display-reset / radio-stop handoff).

## [0.4.0] - 2026-04-11

### Added

- ELECROW ThinkNode-M2 board support (ESP32-S3 + SX1262 + 1.3" SH1106 OLED
  via CP2102 UART bridge, 1000 mAh Li-Po handheld)
  - Pin map verified against Meshtastic's
    `variants/esp32s3/ELECROW-ThinkNode-M2`
  - DIO3 TCXO driven at 3.3 V (not 1.8 V like Heltec)
  - Dedicated `LORA_POWER_EN` (GPIO48) sequenced HIGH before SPI init
  - `VEXT_ENABLE` (GPIO46) is active HIGH — opposite polarity from Heltec
  - Reuses the existing async SH1106 driver at I2C address 0x3C

### Changed

- `hal::esp32s3::{init_spi, init_i2c}` and `board::esp32s3::init_radio` are
  now generic over GPIO type, so ESP32-S3 boards with different pin
  assignments can share the helpers.
- `board::esp32s3::init_radio` takes a `TcxoCtrlVoltage` parameter instead
  of hardcoding `Ctrl1V8` — needed for ThinkNode-M2's 3.3 V TCXO.
- SSD1306 display items in `board::esp32s3` are now cfg-gated to Heltec
  boards (ThinkNode-M2 supplies its own SH1106 `create_display`).
- `justfile` uses `cargo +esp` for Xtensa targets instead of `cargo +nightly`
  with a `RUSTC=esp` override — the self-contained esp toolchain keeps its
  cargo, rustc, and rust-src versions aligned. Public nightly's rust-src
  had drifted ahead of the esp fork, breaking `-Zbuild-std`.
- Migrated to embassy-executor 0.10 and embassy-rp 0.10
- ESP crates (esp-hal, esp-rtos, esp-backtrace, esp-println) patched from
  esp-rs/esp-hal main branch until esp-rtos publishes with 0.10 support
- Adapted to embassy-executor 0.10 SpawnToken API (task functions now return
  `Result<SpawnToken, SpawnError>`, `Spawner::spawn` no longer returns Result)
- Adapted to esp-hal main branch API changes: `efuse::base_mac_address()`,
  `esp_rtos::start` now requires `SoftwareInterrupt`, embassy-rp SPI
  constructor now requires DMA interrupt bindings
- `just check` now runs clippy with `-D warnings` instead of plain `cargo check`
- Rust toolchain bumped to 1.94 in mise.toml

### Fixed

- `core::mem::forget(output)` pattern in ESP32-S3 board files is now
  `let _vext = Output::new(...)` — current esp-hal makes `Output` non-Drop,
  which made `forget` a no-op and tripped clippy's `forget_non_drop` lint.
- UART boards (heltec_v3_uart) display stuck on splash screen after boot.
  Host task sent `DisplayCommand::Reset` (which sets disconnected=true) instead
  of `DisplayCommand::On`. UART has no DTR, so nothing ever cleared it.

## [0.3.0] - 2026-04-10

### Added

- Waveshare RP2040-LoRa board support (RP2040 + SX1262, no OLED, LED on GPIO25)
  - First RP2040 board — third MCU family after ESP32-S3 and nRF52840
  - New `hal/rp2040.rs` module with MCU primitives (SPI, USB, flash unique ID)
  - USB-C via FPC ribbon cable to adapter board
  - PE4259 antenna switch on GPIO17, 850-930 MHz (HF)

### Changed

- `cortex-m-rt/set-vtor` moved from global dependency to per-board feature
  (Cortex-M0+ has no VTOR register)
- `build.rs` memory.x selection is now target-aware (thumbv6m vs thumbv7em)

## [0.2.1] - 2026-04-07

### Fixed

- Display stuck on status screen after USB disconnect (stale radio status
  updates re-rendered dashboard over splash because `disconnected` flag was
  not set in the `Reset` handler)
- Radio left running after host disconnect, now sends `StopRx` when DTR drops
- System deadlock when host app closes serial port while radio is receiving
  (unsolicited responses in radio task now use `try_send` to avoid blocking
  on a full response channel)
- LED orphaned if OLED display initialization fails (display task now falls
  back to a LED-only event loop instead of returning early)

## [0.2.0] - 2026-04-07

### Added

- Wio Tracker L1 board support (nRF52840 + SX1262, SH1106 OLED)
- LED blink on packet activity: brief flash on RX and TX when display is on
  - RX brightness scales with SNR (stronger signal = brighter)
  - Works on all boards with a user LED (Heltec V3/V4, Wio Tracker)
- Native USB CDC-ACM support for Heltec V3 (requires hardware mod: solder R29/R3)
- `heltec_v3_uart` feature for stock (unmodified) V3 boards via CP2102 bridge
- `RgbLed` trait and `SimpleLed` driver for GPIO-based LEDs
- `LoRaBoard` trait with full associated types (`RadioParts`, `CommParts`,
  `DisplayParts`, `DisplayDriver`, `LedDriver`) and `BoardParts<R, C, D, L>` struct

### Changed

- Major firmware architecture reorganization:
  - `hal/` module for MCU-specific primitives (ESP32-S3, nRF52840)
  - `driver/` module for hardware peripheral drivers (SH1106, SimpleLed)
  - `host/` module for unified USB/UART communication with internal cfg dispatch
  - `board/` contains only board definitions — no MCU helpers or drivers mixed in
  - Board files are declarative wiring diagrams calling hal primitives
- `run()` function has zero board-specific conditional compilation
- Display init moved to board layer (`create_display()`) — display task has no cfg
- Flattened `radio/` and `display/` directories (no single-file directories)
- `build.rs` auto-discovers boards by content (implements `LoRaBoard`) and
  helper modules by `use super::` imports — no exclusion lists
- `check` and `clippy` commands now use `--release` profile
- Deduplicated USB/UART protocol framing (shared `CobsDecoder` + `route_command`)
- Removed `ErrorCode::CrcError` (SX1262 silently drops bad-CRC packets)

### Fixed

- Display stuck on splash screen for UART boards (disconnected flag started true)

## [0.1.0] - 2026-03-15

### Added

- Transparent LoRa radio over USB CDC-ACM (VID `1209`, PID `5741`)
- COBS-framed fixed-size LE binary protocol (9 commands, 7 responses)
- Radio configuration: frequency, bandwidth, spreading factor, coding rate, TX power
- Continuous RX mode with RSSI/SNR per packet
- Single-shot TX with optional per-packet config override
- SSD1306 OLED display support with status dashboard
  - Radio state, frequency, modulation parameters
  - Packet counters (RX/TX)
  - RSSI sparkline (1-minute history, TX shown as dotted bars)
  - Combined splash/waiting screen on disconnect
- Auto-detect USB host connect/disconnect via DTR
- Config validation against per-board hardware limits
- `TX_POWER_MAX` sentinel (`-128`): auto-use the board's maximum TX power
- Board support: Heltec V3, Heltec V4, RAK WisBlock 4631
- Python host library (`donglora` on PyPI) with `pip install` and `donglora-mux` CLI
- USB multiplexer for sharing one dongle with multiple applications
- Two-way LoRa bridge over TCP (`examples/lora_bridge.py`)

### Technical

- Embassy async runtime (no threads, CPU sleeps when idle)
- Zero-config: boots and waits for host commands
- Protocol-agnostic: firmware has no opinion about what you transmit
- Tasks never panic: all errors reported to host or logged
- Board abstraction: one `.rs` file + one Cargo feature per board
