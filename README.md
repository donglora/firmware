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

| Board | MCU | Radio | Target |
|-------|-----|-------|--------|
| Heltec V3 | ESP32-S3 | SX1262 | xtensa-esp32s3-none-elf |
| Heltec V4 | ESP32-S3 | SX1262 | xtensa-esp32s3-none-elf |
| RAK WisBlock 4631 | nRF52840 | SX1262 | thumbv7em-none-eabihf |

## Adding a Board

See [src/board/PORTING.md](src/board/PORTING.md).

## Protocol

See [PROTOCOL.md](PROTOCOL.md) for the complete wire format specification.
