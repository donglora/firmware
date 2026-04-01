# Porting DongLoRa to a New Board

## Steps

### 1. Create the board file

Create `src/board/your_board.rs`. Use an existing board as a template:
- ESP32-S3 boards: copy `heltec_v4.rs` (uses shared helpers from `esp32s3.rs`)
- nRF52840 boards: copy `rak_wisblock_4631.rs`

ESP32-S3 boards share common type aliases, peripheral bundles, and init helpers
via `src/board/esp32s3.rs`. Your board file only needs the trait impl and
board-specific init (Vext GPIO, display reset). If you add a new ESP32-S3 board,
add its feature to the `#[cfg(any(...))]` gate on `mod esp32s3` in
`src/board/mod.rs.j2`.

### 2. Define concrete types

Your board must export these type aliases:

```rust
pub type RadioDriver = ...; // Must work with lora_phy::LoRa<RadioDriver, Delay>
pub type UsbDriver = ...;   // Must implement embassy_usb_driver::Driver<'static>
pub type DisplayI2c = ...;  // Must implement embedded_hal_async::i2c::I2c
```

### 3. Define peripheral bundles

```rust
pub struct RadioParts {
    pub driver: RadioDriver,
    pub delay: embassy_time::Delay,
}

pub struct UsbParts {
    pub driver: UsbDriver,
}

pub struct DisplayParts {
    pub i2c: DisplayI2c,
    pub mac: [u8; 6],
}
```

### 4. Implement the LoRaBoard trait

```rust
use super::traits::LoRaBoard;

impl LoRaBoard for Board {
    const NAME: &'static str = "Your Board Name";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22); // check your radio + PA
    fn init() -> Self { ... }
    fn mac_address() -> [u8; 6] { ... } // read from efuse, FICR, etc.
}
```

The compiler will refuse to build if any of these are missing.

### 5. Implement `Board::into_parts()`

```rust
impl Board {
    pub fn into_parts(self) -> (RadioParts, UsbParts, Option<DisplayParts>) {
        // Initialize SPI, DMA, radio, USB, I2C
        // Return peripheral bundles
    }
}
```

### 6. Add Cargo feature

In `firmware/Cargo.toml`, add a feature with your board's HAL dependencies:

```toml
[features]
your_board = ["dep:your-hal", ...]
```

### 7. Add to justfile

Add your board's feature/target/chip to the `boards` list at the top of `justfile`.

### 8. Build and test

```sh
just check your_board    # Must compile
just clippy your_board   # Must be clean
just build your_board    # Produces firmware
```

`build.rs` auto-discovers your `.rs` file — no template editing needed.
