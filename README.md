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
| Heltec V3 | `heltec_v3` | ESP32-S3 | SX1262 | Native CDC-ACM | xtensa-esp32s3-none-elf |
| Heltec V3 (UART) | `heltec_v3_uart` | ESP32-S3 | SX1262 | CP2102 bridge | xtensa-esp32s3-none-elf |
| Heltec V4 | `heltec_v4` | ESP32-S3 | SX1262 | Native CDC-ACM | xtensa-esp32s3-none-elf |
| RAK WisBlock 4631 | `rak_wisblock_4631` | nRF52840 | SX1262 | Native CDC-ACM | thumbv7em-none-eabihf |
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

See [PROTOCOL.md](PROTOCOL.md) for the complete wire format specification.
