# Phase 1 — Reconnaissance

## Project

`donglora` v1.5.2 — Embedded Rust firmware. Single binary per board, compiled with mutually-exclusive cargo features. Exposes the LoRa radio over a host transport (USB CDC-ACM or UART) using a COBS-framed binary protocol (DongLoRa Protocol v2).

## Languages / runtime

- **Rust 1.94 stable** for ARM (thumbv6m, thumbv7em); **Rust esp** fork for Xtensa (esp32, esp32-s3). `no_std`, `no_main`, `panic = abort`, LTO fat, opt-level 3.
- **No allocator.** All collections are fixed-capacity `heapless::Vec<_, N>`.
- **Async runtime: embassy-executor 0.10** (Cortex-M) / **esp-rtos 0.3** (Xtensa). Three long-lived tasks per board.
- **Statecharts: hsmc 0.6** drive the host, radio, and display tasks.

## Supported boards (one feature → one binary)

| Board                       | MCU        | Radio  | Host transport         |
|-----------------------------|------------|--------|------------------------|
| heltec_mesh_node_t114       | nRF52840   | SX1262 | USB CDC-ACM (native)   |
| heltec_v3                   | ESP32-S3   | SX1262 | USB CDC-ACM (native)   |
| heltec_v3_uart              | ESP32-S3   | SX1262 | UART0 via CP2102       |
| heltec_v4                   | ESP32-S3   | SX1262 | USB CDC-ACM (native)   |
| elecrow_thinknode_m2        | ESP32-S3   | SX1262 | UART0 via CP2102       |
| lilygo_tbeam                | ESP32      | SX1276 | UART0 via CP2102       |
| lilygo_tbeam_supreme        | ESP32-S3   | SX1262 | USB CDC-ACM (native)   |
| rak_wisblock_4631           | nRF52840   | SX1262 | USB CDC-ACM (native)   |
| waveshare_rp2040_lora       | RP2040     | SX1262 | USB CDC-ACM (native)   |
| wio_tracker_l1              | nRF52840   | SX1262 | USB CDC-ACM (native)   |

## Entry points (untrusted-byte ingress)

There is **exactly one network-facing entry point per board image**: the host transport (USB CDC-ACM **or** UART, never both in the same build).

| Module                       | Inbound bytes from                  |
|------------------------------|-------------------------------------|
| `src/host/usb.rs`            | USB CDC-ACM (host OS, kernel-side)  |
| `src/host/uart.rs`           | UART0 (CP2102 → host OS)            |
| `src/radio.rs` (RX path)     | LoRa over-the-air (any peer in RF range, on configured freq+sync) |

There is **no networking** (TCP/UDP/HTTP/etc.), **no filesystem**, **no shell**, **no storage**. The radio is the only RF surface; OTA RX packets become a `DeviceMessage::Rx(...)` forwarded to the host with `tag = 0` — the firmware itself does not interpret payload bytes.

USB descriptors / VID:PID `1209:5741` (composite IAD device). The CDC-ACM serial-number string is the device's MAC address as 12-char uppercase hex (`src/host/usb.rs:184-189`).

## Wire format (one path from byte → command)

```
raw bytes
  → FrameDecoder (donglora-protocol 1.2.0 frame.rs)
      COBS-delimited, CRC16/CCITT-FALSE-protected, payload ≤ MAX_PAYLOAD_FIELD (275 B)
  → dispatch_frame (host/framing.rs)
      rejects tag=0 with async ERR(EFRAME)
      calls Command::parse(type_id, payload)
  → InboundEvent::Command { tag, cmd } via CommandChannel (depth 8)
  → radio task statechart (radio.rs)
```

Egress (`DeviceMessage::Ok|Err|Rx|TxDone`) is symmetric: built by the radio task, sent through `OutboundChannel` (depth 64), encoded by `encode_outbound`, written packet-by-packet.

Commands accepted from host: `PING`, `GET_INFO`, `SET_CONFIG`, `TX`, `RX_START`, `RX_STOP` (only these `type_id` bytes; anything else → `ERR(EUNKNOWN_CMD)`).

## Trust boundaries

| Boundary                              | Direction       | Treated as       |
|---------------------------------------|-----------------|------------------|
| USB / UART host → firmware            | inbound bytes   | **trusted** (it's the user's PC) |
| LoRa RF peer → firmware (RX payload)  | inbound bytes   | **untrusted**, but only memcpy'd to host — firmware never interprets |
| firmware → SX1262/SX1276 via SPI      | outbound        | trusted (own peripheral) |

The firmware is intentionally a **transparent radio bridge**: there is no auth, no crypto, no session state, no key material on device. The radio's freq, power, sync_word, etc. are whatever the host sets (within hardware limits enforced by `validate_lora`).

## Auth / session layer

**None.** Anything that can write CDC-ACM bytes to the device can drive the radio. Per design — the device is a "dongle," ownership is physical-attachment.

The protocol's inactivity watchdog (`PROTOCOL.md §3.4`, 1 s) drops the radio to `Unconfigured` if the host stops sending bytes — this is liveness/cleanup, **not** auth. USB-side this is gated on DTR (USB CDC line state); UART has no DTR so it relies purely on the watchdog.

## Data at rest

**None of security interest.**
- Static `Info` blob built once at boot — MAC, fw version, capability bitmaps. Visible to anyone who issues `GET_INFO`.
- No credentials, tokens, keys, PII.
- Persistent storage: only board-specific bootloader settings region; **firmware never writes to flash at runtime** (see `ld/*.x` — app region is read-only post-flash).
- nRF52840 boards' `s140 SoftDevice` region (0x01000–0x26000) preserved at flash time; firmware never touches it.

## Dependencies (Cargo.lock, security-relevant subset)

- `donglora-protocol 1.2.0` — own crate, the wire codec (frame, command, modulation, info, events). Fuzz/mutation-tested in sibling repo (`/home/swaits/Code/donglora-org/protocol/`).
- `ucobs 0.3` — COBS encode/decode, no_std, very small.
- `lora-phy` — git fork from `swaits/lora-rs branch=fixed` (carries CAD IRQ-clear fixes pending upstream PRs #428/#431/#432). All SX126x/SX127x SPI primitives.
- `hsmc 0.6` — own crate (statecharts).
- `embassy-* 0.5/0.6/0.10/0.8` — well-known embedded async runtime.
- `defmt 0.3`, `panic-probe 1.0` (Cortex-M boards) / `esp-backtrace 0.19` + `esp-println 0.17` (Xtensa boards).
- `heapless 0.9` — fixed-capacity collections.

No HTTP, JSON, TLS, SQL, template, regex, deserialization-of-user-data dependencies.

## Deployment context

Firmware is flashed via UF2 (Cortex-M) or `espflash` (Xtensa). No OTA update path on the device itself — re-flashing requires physical USB. Production builds run with `DEFMT_LOG` unset → defmt strips info/debug/warn at compile time; only `error!` survives.

## Existing security controls / hardening already in place

- **`#![cfg_attr(not(test), no_std)]` / `panic = abort`** — no panic unwinding to abuse.
- **`#![deny(clippy::undocumented_unsafe_blocks)]`** in Cargo.toml — every `unsafe` block must justify itself.
- **`unsafe` audit** (4 sites total):
  - `main.rs:105` — one-time `static mut INFO` init at boot, before any task spawns.
  - `hal/nrf52840.rs:102, 140, 151` — `read_volatile` / `write_volatile` on MMIO registers (POWER.RESETREAS, FICR.DEVICEADDR). Standard.
  - `hal/rp2040.rs:68, 79` — one-time `static mut MAC_REF` init at boot.
- **Memory layout (`ld/nrf52840-memory.x`)** preserves the factory SoftDevice region; UF2-only flashing prevents accidental overwrite.
- **CRC-16/CCITT-FALSE** on every frame, COBS framing — corrupt bytes are caught by either layer before reaching `Command::parse`.
- **Strict typed parsing**: every enum field rejects out-of-range bytes (`from_u8` returns `Option` and `InvalidField` is propagated); reserved bits in `TxFlags` rejected; reserved byte in `LrFhssConfig` rejected.
- **`validate_lora` (`radio.rs:2125-2145`)**: freq within board's chip-supported range, SF ∈ {5..=12} **and** in board bitmap, BW in board bitmap, TX power within board range, preamble ≥ 6.
- **Hardware-limit clamping**: `MAX_OTA_PAYLOAD = 255` matches the chip's `payload_len` register width; the receive buffer is sized exactly to it.
- **`RADIO_THROTTLED` backpressure flag** prevents queue floods if the host is unresponsive.
- **State-machine cancel-safety**: cancel-unsafe lora-phy futures (`lora.tx`, `lora.cad`, `LoRa::new`, `lora.init`) are hosted in chart states whose only event handlers are emitted by the future itself on natural completion — chart cannot drop them.
- **Reset-and-recovery state machine** with bounded attempts (`MAX_RECOVERY_ATTEMPTS = 4`) and a terminal `Dead` state that still answers `PING`/`GET_INFO`.
