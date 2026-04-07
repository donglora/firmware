# Porting DongLoRa to a New Board

## Steps

### 1. Create the board file

Create `src/board/your_board.rs`. Use an existing board as a template:
- ESP32-S3 boards: copy `heltec_v4.rs` (uses shared helpers from `board/esp32s3.rs` and `hal/esp32s3.rs`)
- nRF52840 boards: copy `rak_wisblock_4631.rs` (uses shared helpers from `hal/nrf52840.rs`)
- New MCU family: create a new `hal/<mcu>.rs` with MCU primitives first

`build.rs` auto-discovers your board file — any `.rs` file in `src/board/` that
implements `LoRaBoard for Board` is automatically detected as a board. Helper
modules (like `esp32s3.rs`) are auto-discovered via `use super::<helper>` imports
in board files. No template editing or exclusion lists needed.

### 2. Define concrete types

Your board must export these type aliases:

```rust
pub type RadioDriver = ...; // Must work with lora_phy::LoRa<RadioDriver, Delay>
pub type UsbDriver = ...;   // Must implement embassy_usb_driver::Driver<'static>
                             // (or UartDriver for UART boards)
pub type DisplayI2c = ...;  // Must implement embedded_hal_async::i2c::I2c
pub type DisplayDriver = ...; // Must implement DrawTarget<Color = BinaryColor>
```

### 3. Define peripheral bundles

```rust
pub struct RadioParts {
    pub driver: RadioDriver,
    pub delay: embassy_time::Delay,
}

pub struct UsbParts {  // or UartParts for UART boards
    pub driver: UsbDriver,
}

pub struct DisplayParts {
    pub i2c: DisplayI2c,
}
```

### 4. Implement the LoRaBoard trait

```rust
use super::traits::{BoardParts, LoRaBoard};

impl LoRaBoard for Board {
    const NAME: &'static str = "Your Board Name";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22); // check your radio + PA

    type RadioParts = RadioParts;
    type CommParts = UsbParts;       // or UartParts
    type DisplayParts = DisplayParts;
    type DisplayDriver = DisplayDriver;

    fn init() -> Self { ... }
    fn mac_address() -> [u8; 6] { ... } // read from efuse, FICR, etc.
    fn into_parts(self) -> BoardParts<RadioParts, UsbParts, DisplayParts> {
        // Initialize buses via hal::, construct drivers, return BoardParts
        BoardParts { radio, host, display: Some(display_parts), mac: Self::mac_address() }
    }
}
```

### 5. Add display init

```rust
pub async fn create_display(i2c: DisplayI2c) -> Option<DisplayDriver> {
    // Construct and initialize your display driver (SSD1306, SH1106, etc.)
}
```

### 6. Add Cargo feature

In `firmware/Cargo.toml`, add a feature with your board's HAL dependencies:

```toml
[features]
your_board = ["dep:your-hal", ...]
```

### 7. Add to justfile

Add your board's feature/target/chip to the board definitions at the top of `justfile`.

### 8. Build and test

```sh
just check your_board    # Must compile
just clippy your_board   # Must be clean
just build your_board    # Produces firmware
```
