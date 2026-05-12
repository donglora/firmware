# Phase 2 — Threat model

## Actors

| Actor                                | Capabilities                                                        | Reach              |
|--------------------------------------|---------------------------------------------------------------------|--------------------|
| **Local USB attacker (LUSB)**        | Read/write raw bytes to CDC-ACM endpoint or UART pins; read USB descriptors. Implies kernel-level access on the host *or* physical hardware access. | high impact, but presumes the host is already compromised |
| **OTA peer (OTA)**                   | Transmit arbitrary LoRa packets within RF range, on the configured frequency / sync word / SF / BW. Cannot read or affect host transport directly. | low impact (firmware does not interpret), broad reach |
| **Hostile host application (APP)**   | Subset of LUSB: a malicious userspace program on the host PC that opens the CDC-ACM device — possibly without the user's intent. Limited to whatever the OS permits on that device node. | depends on host's permissions on the tty/dev node |
| **Build-time adversary (BUILD)**     | Compromised crate in the dependency graph or compromised local toolchain. | out of scope of runtime audit; noted only for completeness |

## Entry points and worst-case impact, in attacker-reachability order

### Entry point #1 — Host transport (USB CDC-ACM / UART)

**Reachable by:** LUSB, APP.

**What they can influence:** every byte of every frame after `0x00` sentinels. Specifically: `type_id` (1 B), `tag` (2 B LE), `payload` (≤ 275 B), `crc` (2 B). After CRC + COBS + parser checks, controlled fields are:
- `type_id` ∈ {1..=6} (validated by `Command::parse`)
- `tag` ∈ 1..=65535 (tag=0 rejected by framing layer)
- For `SET_CONFIG`: every byte of `LoRaConfig` (15 B) — freq, SF, BW, CR, preamble_len, sync_word, tx_power, header_mode, payload_crc, iq_invert. The other three modulations parse but the firmware rejects with `EMODULATION` after parsing.
- For `TX`: `TxFlags` byte (bit 0 only — reserved bits rejected) + 1..=255 bytes of OTA payload.

**Worst-case if attacker fully controls input:**

The firmware is a **transparent bridge by design**. Any attacker who can write to the host transport can:
- Tune the radio to any frequency in the chip's range (e.g. 137 MHz – 1020 MHz for SX127x; 150–960 MHz for SX126x).
- Set TX power up to the board's `TX_POWER_RANGE.max` (typically +22 dBm).
- Transmit arbitrary packet bytes with arbitrary sync word at arbitrary spreading factor.
- Receive on arbitrary parameters.
- Read the device's MAC address (via `GET_INFO` and/or USB serial descriptor).

This is **the intentional product feature**. The firmware does not enforce regional frequency rules, duty cycle, listen-before-talk policy beyond the SF-tuned CAD, or any auth: the host owns the radio. Treating "host can drive the radio" as a vulnerability would be a category error.

The remaining attacker goals against the firmware itself are:
- **DoS** — crash, hang, reboot, or wedge the firmware so the user has to power-cycle.
- **Memory corruption / RCE** — overflow a buffer, hit an OOB write, leak ASLR-less code-pointer.
- **Cross-device escalation** — pivot from the radio task into the host PC (e.g. malicious USB descriptors / report).
- **Information leak** — exfiltrate cross-tenant data. But there is no cross-tenant data on this device. Skip.

### Entry point #2 — Over-the-air RX

**Reachable by:** OTA. Anyone with a LoRa radio that can transmit on the configured (`freq`, `sf`, `bw`, `sync_word`, `cr`, `preamble_len`, `iq_invert`) tuple while the firmware is in `RxActive`.

**What they can influence:** the bytes of a single received LoRa packet — `0..=MAX_OTA_PAYLOAD` (255) bytes of body. Side-channel metadata that the firmware *generates* (not the attacker): RSSI (signed i16), SNR (signed i16), timestamp (u64 micros). The header_mode / CRC flags are configured by the host, not the OTA peer.

**What this eventually does:** `collect_rx_packet` (`radio.rs:1664-1702`) calls `lora.get_rx_result(pkt, &mut ctx.rx_buf)`. Length is clamped to `MAX_OTA_PAYLOAD` before `extend_from_slice` into a fresh `heapless::Vec<u8, 255>`. The vec becomes the body of a `DeviceMessage::Rx`. Wire encoding (`events.rs:228-249`) is a fixed-layout memcpy, no parsing of OTA bytes inside the firmware.

**Worst-case if attacker fully controls the OTA packet:**
- The firmware does not interpret OTA payload bytes; they are an opaque buffer relayed verbatim to the host. Worst is a memory-corruption bug in `get_rx_result` (lora-phy crate) or in `extend_from_slice` (heapless), which would be a third-party bug not firmware logic.
- Spam packets at the configured peer ID to flood `OutboundChannel` (depth 64). Throttled by `RADIO_THROTTLED` once host stalls; otherwise the host sees a burst. Standard RF DoS — there is no defense at the device level beyond "host stops listening."
- Trigger `IrqWaitFailed` → `RxResuming` retry loop. After `MAX_RESUME_RETRIES = 5` falls back to `Idle`. Recovers cleanly.

OTA is a **broad reach but very narrow impact** surface: the firmware exposes essentially no interpretation surface to the OTA peer.

### Entry point #3 — USB descriptor handling (control transfers)

**Reachable by:** LUSB, APP (anyone enumerating the device).

**What they can influence:** USB control transfer request type / index / length. Handled inside `embassy_usb::Builder::run()` — out of firmware's code.

**Worst-case:** a malformed control transfer crashes the USB stack and the firmware. The `CTRL_BUF` size was bumped to 128 specifically because 64 was crashing on some Linux enumeration paths (see comment in `host/usb.rs:191-196`). The user reports modem-manager's `SET_INTERFACE` with out-of-range alt setting is normal and embassy_usb's STALL is correct. **Suspected** dependency-level risk that we accept as a third-party concern.

## Code paths that warrant deep audit (prioritized)

In rough descending order of attacker-reachability × impact:

1. **Frame decode → command dispatch (`host/framing.rs`, `host/usb.rs::dispatch_bytes`, `host/uart.rs::dispatch_bytes`, `donglora-protocol/frame.rs`, `commands.rs`, `modulation.rs`):** every reachable byte from LUSB flows through here. Look for panics, buffer overruns in fixed-capacity scratch buffers, ambiguous overflow / CRC-skip paths.

2. **`Command::parse` for `SET_CONFIG`:** the deepest parse path — modulation-id dispatch, then nested decode. `LoRaConfig::decode` and friends.

3. **`apply_set_config` + `validate_lora` (`radio.rs:1233-1309`, `2125-2145`):** post-parse validation. Where do unchecked fields go — sync_word, iq_invert, header_mode, payload_crc? Memory note flags sync_word low-nibble validation as intentionally absent.

4. **`Command::Tx` ingestion → on-air TX (`radio.rs` chart `TxSession`):** TX payload (1..=255 attacker bytes) reaches the SX1262 PA without further parsing. The body bytes never act on firmware state except as opaque memcpy. Audit for buffer-length confusion between the chart's `pending_tx.data` and lora-phy's `prepare_for_tx(.., &pt.data)`.

5. **`get_rx_result` → `record_packet` → `OutboundFrame` encode (`radio.rs:1664-1715`, `2008-2040`; `events.rs:228-289`):** the only path where OTA-attacker bytes are ingested. Audit length handling and RxPayload encoding's bounds.

6. **State machine reachability**: chart panics (`tag_or_panic`, `NonZeroU16::new(...).expect(...)`, `radio chart returned`) — are any reachable from wire input?

7. **`unsafe` blocks** (4 sites): are they sound under the stated preconditions?

8. **USB serial number / Info disclosure** — what does the device disclose without prior auth?

9. **Dependencies**: any reachable CVE-bearing dep? Check `donglora-protocol`, `ucobs`, `lora-phy` (fork), `hsmc`, `heapless`, `embassy-usb`, `embassy-nrf` / `embassy-rp` / `esp-hal`.

10. **Build-time (`build.rs`)**: code generation reads files from `src/board/`. Build-time only; out of runtime attacker scope.

## Out-of-scope explicitly

- The host-side application that consumes the protocol (out of repo: ai-bot, bridge, client-py, mux-rs).
- The Anthropic / Claude scripting context.
- The donglora-protocol crate's *fuzz-targeted* invariants — they are validated in the sibling repo (`/home/swaits/Code/donglora-org/protocol/fuzz/`); we audit only how the firmware *uses* the crate.
- Physical-side-channel attacks (power, timing, EM).
- RF-regulatory compliance (firmware is intentionally transparent — see above).
