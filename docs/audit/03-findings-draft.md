# Phase 3 — Targeted analysis (raw notes)

Walked through every category in MYTHOS Phase 3 against the firmware's actual code paths. Most categories are not applicable to no_std embedded LoRa firmware with no networking, filesystem, shell, database, templates, deserializers, crypto, or auth. The categories that *do* apply have been traced in full below.

## Categories N/A by construction

- **SQL / NoSQL injection** — no database, no query language anywhere.
- **Command injection** — no `exec` / `spawn`; the firmware has no shell.
- **Template / SSTI** — `minijinja` is used only in `build.rs` at build time on a developer-controlled template (`src/board/mod.rs.j2`). Not exposed to runtime input.
- **LDAP / XPath / EL** — none.
- **Header injection / CRLF / log injection** — no HTTP, no syslog, no user-controlled string output.
- **Authentication / Authorization** — none exists; firmware does not implement auth (design choice, see threat model).
- **Cryptography** — none used; CRC-16/CCITT-FALSE is integrity-only, not security.
- **Deserialization** — no serde, pickle, yaml, marshal, BinaryFormatter, or other reflective deserializer. Every field is decoded byte-by-byte via fixed-offset slicing in `donglora-protocol`.
- **XSS / CSRF / CORS / Open redirect / Prototype pollution / ReDoS** — no JS, no HTML, no regex, no URL routing.
- **SSRF** — no outbound HTTP / DNS / proxy. The "outbound URL fetcher" analog would be `lora.tx()`, but the host fully controls every parameter by design.
- **File operations / Zip slip / Symlink** — no filesystem at runtime.
- **XML** — no XML parser.

## Categories that DO apply (and the traces)

### Memory safety in parsing pipelines

Walked from `host_task` byte ingress to every consumer:

**Frame decode (`donglora-protocol-1.2.0/src/frame.rs`):**
- `FrameDecoder::buf: [u8; MAX_COBS_ENCODED = 283]` — bounded.
- Index `self.len` only ever written via `if self.len < self.buf.len() { ... self.len += 1 }` — safe.
- On overflow → `overflowed = true; len = 0` — bytes are discarded silently until the next `0x00` sentinel resyncs; never out-of-bounds.
- On `0x00` → `let mut decoded = [0u8; MAX_PRE_COBS_FRAME = 280]; ucobs::decode(&self.buf[..self.len], &mut decoded)` returns `Option<usize>`; None → emit `FrameDecodeError::Cobs` and reset.
- `emit_decoded` requires `decoded.len() >= FRAME_HEADER_SIZE + FRAME_TRAILER_SIZE` (5 B); checks CRC; only then slices `type_id / tag / payload` from fixed offsets.

**Confirmed safe**: no path to an OOB read/write; no panic; no unbounded allocation.

**Command parse (`donglora-protocol-1.2.0/src/commands.rs::Command::parse`):**
- `TYPE_TX`: requires `!payload.is_empty()`; reads `payload[0]` for flags; rejects reserved bits via `TxFlags::from_byte`; rejects `body.is_empty()`; rejects `body.len() > MAX_OTA_PAYLOAD`; `extend_from_slice` is bounded by both the slice length and the heapless capacity (`MAX_OTA_PAYLOAD = 255`).
- `TYPE_SET_CONFIG`: every field byte feeds an `Option`-returning `from_u8` (rejects out-of-range), or a `match { 0 => false, 1 => true, _ => Err }` (rejects non-boolean), or an explicit length check `if buf.len() != WIRE_SIZE { Err }`.
- All other types require empty payload.

**Confirmed safe.**

**`validate_lora` (`src/radio.rs:2125-2145`):** numeric range checks on `freq_hz`, `sf`, `bw`, `tx_power_dbm`, `preamble_len`. Shift-by-`sf` is safe (`sf` already bounded ≤12 by the preceding check, so `1u16 << sf ≤ 4096`); shift-by-`bw.as_u8()` is safe (`bw` enum repr is ≤ 13, `1u16 << 13 = 8192`).

Not validated: `preamble_len` upper bound, `sync_word`, the post-parser-checked `header_mode`/`payload_crc`/`iq_invert`.

- `preamble_len = 65535` — fed to `lora.create_*_packet_params`. `toa_us` math (`src/lbt.rs:44-77`) uses u64 throughout and clamps to `u32::MAX`. With SF12/BW7.81 kHz/preamble 65535: `t_preamble_us = 65535 * 524520 ≈ 3.4e10` which is u64-safe and gets clamped to `u32::MAX = 4.29e9`. **No overflow.** The radio chip's HW register handles whatever value is loaded; reasonable lora-phy behavior expected.
- `sync_word = anything` — the memory note flags this as intentional (matches RadioLib/Semtech precedent). Confirmed not a vulnerability.

**Confirmed safe.**

**`try_start_rx_hw` / `cache_rx_pkt` (`src/radio.rs:1348+, 1628+`):** `MAX_OTA_PAYLOAD as u8 = 255` — current `MAX_OTA_PAYLOAD = 255` exactly fits `u8::MAX`. If the constant were ever raised above 255 the `as u8` would silently truncate — but the SX126x chip register is one byte wide so the constant is hardware-bound. No current bug.

**Confirmed safe.**

**`collect_rx_packet` (`src/radio.rs:1664-1702`):**
- `rx_buf: [u8; MAX_OTA_PAYLOAD]` — the buffer handed to `lora.get_rx_result(pkt, &mut rx_buf)`.
- The chip's `RX_PLD_LENGTH` is `255` (set in `try_start_rx_hw`), and the chip hardware register is one byte wide → `len` returned is bounded by 255 at the silicon level.
- `let copy_len = (len as usize).min(MAX_OTA_PAYLOAD)` — defensive clamp.
- `data.extend_from_slice(&ctx.rx_buf[..copy_len])` into `HVec<u8, MAX_OTA_PAYLOAD>` — bounds-safe.

**Confirmed safe.**

### Panic reachability from wire input

Catalog of panic sites and reachability:

| Location | Trigger | Reachable from wire? |
|---|---|---|
| `radio.rs:937` `NonZeroU16::new(tag).expect(...)` for `InboundEvent::Command` | tag=0 reaches radio task | No — `host/framing.rs:36-40` rejects tag=0 with `ERR(EFRAME)` before forwarding to `commands` channel |
| `radio.rs:1209-1213` `tag_or_panic` | Action reads `incoming_tag` when None | No — actions that read it only run after `next_command` has stashed a tag (chart shape) |
| `radio.rs:1753-1754` `PendingTx.tag was 0` | Pending TX with tag=0 | No — same framing layer rejects tag=0 |
| `radio.rs:2232-2235` `radio chart returned` | Chart `run()` returns | No — chart is non-terminating (all leaf states have `during:` activities; this would require a chart-design bug, not a wire input) |
| `main.rs:108, 113, 125, 138, 145` `.expect("…")` | Spawn / init failure | Boot-time only |
| `host/usb.rs:189` `from_utf8(MAC hex)` | Invalid UTF-8 in MAC hex | Hex chars are constants `0-9 A-F`; always valid ASCII |

**Confirmed: no panic site is reachable from network input.**

### `unsafe` audit

Four `unsafe` blocks in firmware code:

1. `src/main.rs:105` — `INFO = Some(info); INFO.as_ref().expect(...)`. Touched only at boot, in the single-threaded `run(spawner)` before any task is spawned. Subsequent reads via `&'static Info` reference passed by argument. **Sound.**
2. `src/hal/nrf52840.rs:102, 140` — `read_volatile` / `write_volatile` on `0x40000400` (POWER.RESETREAS). Valid MMIO. **Sound.**
3. `src/hal/nrf52840.rs:151` — `read_volatile` on `0x10000000.byte_add(0xA4/0xA8)` (FICR.DEVICEADDR). Valid MMIO. **Sound.**
4. `src/hal/rp2040.rs:68, 79` — `static mut MAC_REF` written once in `init_mac` (single-threaded init), read-only thereafter. **Sound.**

**Confirmed: no unsoundness in declared unsafe blocks.**

### TOCTOU / races

All cross-task communication uses `embassy_sync::channel::Channel` with `CriticalSectionRawMutex` — atomic enqueue/dequeue. Cross-task atomic flags (`RESET_PENDING`, `RADIO_THROTTLED`, `LORA_WEDGED`) use `portable_atomic::AtomicBool` with `AcqRel` swap or `Acquire/Release` loads/stores. The radio chip itself has exactly one user (the radio task), so there is no SPI-bus race.

**No exploitable race observed.**

### Information disclosure

USB serial-number descriptor = MAC address (hex). `GET_INFO` returns same MAC in `mcu_uid`. Either is readable by anyone enumerating the USB device or any successful `GET_INFO`, with no authentication.

- **Status:** Confirmed
- **Severity:** Info (privacy / cross-host tracking)
- **Reasoning:** This is the device's only stable cross-context identifier and it leaks unconditionally. A user reusing the dongle across hosts/networks is trivially correlatable.

### Dependency / supply chain

- `donglora-protocol 1.2.0` — sibling crate, fuzz-tested in `/home/swaits/Code/donglora-org/protocol/fuzz/`. No known CVEs.
- `ucobs 0.3` — small crate; no known CVEs.
- `lora-phy` — `git = "https://github.com/swaits/lora-rs.git", branch = "fixed"`, no `rev=` pin. Cargo.lock pins the current commit by SHA → reproducible *today*. But `cargo update -p lora-phy` will silently advance to whatever is on `branch=fixed` at that moment. If the maintainer's GitHub account is compromised, the next `cargo update` ingests the malicious commit.
- `heapless 0.9`, `embassy-* 0.5/0.6/0.10/0.8`, `defmt 0.3`, `esp-hal 1.1`, `embassy-usb 0.6` — all current versions; no known reachable CVE in their used surface.

The `branch=` (not `rev=`) git dep is the only flag — a **Low** finding (hardening).

### USB stack / control-transfer surface

`embassy_usb` handles control transfers internally. The firmware allocates `CTRL_BUF: [u8; 128]`. A code comment (`host/usb.rs:191-196`) records that the previous 64-byte size matched the panic-site message `"control buffer too small"` in `embassy_usb/src/lib.rs:758` — a real bug observed during enumeration. The 128-byte size is documented as safely above the largest CDC-ACM request.

**Status:** Mitigated; relies on third-party (embassy-usb 0.6) correctness for control-transfer parsing. Not an actionable firmware finding.

### Reliability bug worth noting (not strictly a security finding)

`lilygo_tbeam` (ESP32 classic) — the host transport is UART0 (`GPIO1/GPIO3 → CP2102 → host`). `esp-println` with the `auto` feature falls back to UART0 on ESP32 classic (no USB-Serial-JTAG peripheral). In production builds (`DEFMT_LOG` unset → `error!` only), if the radio chart enters `Dead` or similar `error!`-emitting paths, defmt bytes are emitted onto the same UART as the binary protocol, corrupting frames.

- **Status:** Suspected; not reachable from wire input (the error-level logs fire on hardware faults).
- **Severity:** Low (reliability, not security).
- **Why noted:** Documents a real frame-corruption scenario that an operator should be aware of when soak-testing the T-Beam classic.

For `heltec_v3_uart` and `elecrow_thinknode_m2` (both ESP32-S3, which has USB-Serial-JTAG), esp-println's `auto` selects USB-Serial-JTAG, which is a *different* peripheral than the host UART. Those boards are NOT affected.

### Intentional design choices flagged for transparency

These are NOT findings — they are deliberate, documented features. They appear here so the audit's reader knows they were considered and accepted:

- **No authentication on host transport.** A "dongle" trust model: physical attachment = full access.
- **No RF regulatory enforcement.** Frequency, sync word, and TX power are whatever the host sends (within hardware-supported range). FCC / CE / ETSI compliance is the *user's* responsibility.
- **TX `skip_cad=1` bypasses carrier-sense.** Documented in `PROTOCOL.md`; available to anyone with host-transport access.

## Items considered and dismissed

- Panic-on-malformed-frame (no — framing layer reflects `ERR(EFRAME)` and clears the decoder).
- Integer overflow in `toa_us` (no — u64 intermediates, u32 clamp; bound-checked).
- Buffer overrun on max-payload RX (no — chip HW register caps to 255; firmware buffer is sized to 255; `extend_from_slice` is bounded).
- Frame-decoder buffer exhaustion DoS (mitigated — overflow state silently discards bytes until resync; one packet, one frame; no accumulation across the limit).
- Stack overflow from deep call chains (no recursion; nested stack frames in `dispatch_bytes → feed → emit_decoded → Command::parse → Modulation::decode → LoRaConfig::decode` use ~1.5 KB total on a 4–32 KB task stack budget).
- Memory safety in the unsafe blocks (audited above).
- TOCTOU between USB read and radio dispatch (channel-mediated; no race).
- Side channels (timing on CRC compare, etc.) — irrelevant: there are no secrets to time-leak.
- `panic-probe` emitting panic message: emitted via `defmt-rtt`, which is a *different* hardware channel (SWD probe) than USB/UART. A panic locks the device (DoS) but does not leak to host or peer.
