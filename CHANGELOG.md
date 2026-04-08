# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

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
- Python host library (`clients/python/`) with `pip install` and `donglora-mux` CLI
- USB multiplexer for sharing one dongle with multiple applications
- Two-way LoRa bridge over TCP (`examples/lora_bridge.py`)

### Technical

- Embassy async runtime (no threads, CPU sleeps when idle)
- Zero-config: boots and waits for host commands
- Protocol-agnostic: firmware has no opinion about what you transmit
- Tasks never panic: all errors reported to host or logged
- Board abstraction: one `.rs` file + one Cargo feature per board
