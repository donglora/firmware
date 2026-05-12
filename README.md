# DongLoRa Firmware

Embedded Rust firmware for DongLoRa boards. Exposes the LoRa radio over
USB CDC-ACM using a COBS-framed binary protocol.

## Building

```sh
just setup              # one-time toolchain install
just build heltec_v4    # build for a specific board
just flash heltec_v4    # build + flash
just check-all          # compile-check all boards
just clippy heltec_v4   # lint
just test               # host-side protocol unit tests
```

All commands are run from the `firmware/` directory.

## Supported Boards

| Board | Feature | MCU | Radio | USB | Target |
|-------|---------|-----|-------|-----|--------|
| Heltec Mesh Node T114 | `heltec_mesh_node_t114` | nRF52840 | SX1262 | Native CDC-ACM | thumbv7em-none-eabihf |
| Heltec V3 | `heltec_v3` | ESP32-S3 | SX1262 | Native CDC-ACM | xtensa-esp32s3-none-elf |
| Heltec V3 (UART) | `heltec_v3_uart` | ESP32-S3 | SX1262 | CP2102 bridge | xtensa-esp32s3-none-elf |
| Heltec V4 | `heltec_v4` | ESP32-S3 | SX1262 | Native CDC-ACM | xtensa-esp32s3-none-elf |
| ELECROW ThinkNode-M2 | `elecrow_thinknode_m2` | ESP32-S3 | SX1262 | CP2102 bridge | xtensa-esp32s3-none-elf |
| LilyGo T-Beam | `lilygo_tbeam` | ESP32 | SX1276 | CP2102 bridge | xtensa-esp32-none-elf |
| LilyGo T-Beam S3 Supreme | `lilygo_tbeam_supreme` | ESP32-S3 | SX1262 | Native CDC-ACM | xtensa-esp32s3-none-elf |
| RAK WisBlock 4631 | `rak_wisblock_4631` | nRF52840 | SX1262 | Native CDC-ACM | thumbv7em-none-eabihf |
| Waveshare RP2040-LoRa | `waveshare_rp2040_lora` | RP2040 | SX1262 | Native CDC-ACM | thumbv6m-none-eabi |
| Wio Tracker L1 | `wio_tracker_l1` | nRF52840 | SX1262 | Native CDC-ACM | thumbv7em-none-eabihf |

### Heltec V3 USB Note

The default `heltec_v3` build uses **native USB CDC-ACM**, which requires a
hardware modification to your board. Stock Heltec V3 boards route USB through a
CP2102 bridge chip to UART — the ESP32-S3's native USB pins (GPIO19/GPIO20) are
routed to header pins but not connected to the USB connector.

**To enable native USB on a Heltec V3:**

1. Solder 0-ohm (or 22-ohm) resistors at pads **R29** (D+) and **R3** (D-)
2. Disconnect the CP2102 from the USB connector's D+/D- lines (cut traces or
   remove the CP2102 bridge resistors)

After this mod, the board appears as `/dev/ttyACM0` with the DongLoRa VID:PID
(1209:5741), just like the V4 and nRF boards.

**If you have a stock (unmodified) V3**, use `heltec_v3_uart` instead:

```sh
just build heltec_v3_uart
just flash heltec_v3_uart
```

This builds firmware that communicates via the CP2102 bridge (appears as
`/dev/ttyUSB0`).

## Adding a Board

See [src/board/PORTING.md](src/board/PORTING.md).

## Protocol

See [PROTOCOL.md](https://github.com/donglora/protocol-rs/blob/main/PROTOCOL.md)
in the [donglora-protocol](https://github.com/donglora/protocol-rs) repo
(crates.io: [`donglora-protocol`](https://crates.io/crates/donglora-protocol))
for the complete wire format specification — that is the canonical home;
this firmware just consumes it.

## Trust model and identifiers

DongLoRa is a **transparent LoRa bridge**, not a security boundary:

- **No authentication on the host transport.** Anything that can write
  to the CDC-ACM endpoint or UART pins drives the radio. Physical USB
  attachment is the trust boundary.
- **No RF-regulatory enforcement.** Frequency, sync word, and TX power
  are whatever the host sends, within the chip's hardware range.
  Regional compliance (FCC / CE / ETSI / duty cycle) is the user's
  responsibility, not the firmware's.
- **Device identifier exposed without authentication.** The board's
  6-byte MAC (factory-burned, permanent) is published in two places
  readable by anyone who can enumerate or open the device:
    1. The USB CDC-ACM `iSerialNumber` string descriptor (12 hex
       chars, uppercase). Visible to any OS / userspace process that
       can read USB device descriptors.
    2. The `GET_INFO.mcu_uid` field on the wire, available to any
       host that can issue protocol frames.

  This is intentional — host-side tools (`donglora-mux`,
  `client-py`, etc.) rely on it to distinguish multiple dongles on
  one host. The privacy trade-off: a user reusing the same dongle
  across different hosts/networks is correlatable by that MAC across
  contexts. If that matters in your threat model, isolate the dongle
  per context or run it behind a trusted multiplexer.
