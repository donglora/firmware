# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

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
