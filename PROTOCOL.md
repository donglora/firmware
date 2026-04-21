# DongLoRa Protocol v2 Specification

**Version 1.0**

A minimal, reliable, transport-agnostic wire protocol for exposing a LoRa radio transceiver as a USB-serial (or BLE / WiFi / TCP) device. This document is the normative reference for implementers of both host software and device firmware.

---

## Table of Contents

1. Overview
2. Frame Format
3. State Machine
4. Message Types
5. Common Data Types
6. Message Reference
7. Error Codes
8. Radio Chip ID Enum
9. Capability Bitmap
10. Modulation Parameters
11. Ordering and Interleaving
12. Flow Control and Backpressure
13. Multi-Client Operation (Future)
14. Host Behavior Requirements
15. Device Behavior Requirements
16. Version Compatibility
17. Appendix A — COBS Reference
18. Appendix B — CRC Reference
19. Appendix C — Wire Examples
20. Appendix D — Glossary

---

## 1. Overview

DongLoRa Protocol is the wire protocol spoken between a host application and firmware on a microcontroller attached to a Semtech LoRa transceiver (SX126x, SX127x, SX128x, LR11xx, LR2021, or similar). It exposes the transceiver's native capabilities over a byte-stream transport with no editorialization: the host asks, the chip does.

### 1.1 Design Principles

1. **Minimal.** Ten message types. No sub-protocols, no modes, no sessions.
2. **Transport-agnostic.** Runs identically over USB CDC-ACM, USB-to-UART bridges, plain UART, BLE, TCP, or any other ordered byte stream.
3. **No persistent state.** The device boots unconfigured and forgets everything on host disconnect. There is no device-side memory between sessions.
4. **No policy.** The firmware does not enforce regulatory limits, retry on failure, reorder TX, or apply profiles. The host is authoritative for all policy.
5. **Binary.** No text, no JSON, no AT commands. Every bit is accounted for.
6. **Host drives.** The device never initiates a conversation. Every device-to-host frame is either a response to a command or a radio-sourced event.

### 1.2 Terminology

- **Device**: MCU + radio transceiver + firmware that speaks DongLoRa Protocol.
- **Host**: Any software endpoint speaking DongLoRa Protocol.
- **Client**: In the multi-client future, an individual host connection. A single-client USB deployment has one client, which is the host.
- **Frame**: A single DongLoRa Protocol message on the wire, COBS-encoded and delimited.
- **Tag**: 16-bit correlation identifier for matching responses to commands.
- **CONFIGURED / UNCONFIGURED**: The two states of the device's state machine (Section 3).

---

## 2. Frame Format

### 2.1 Layout

Every DongLoRa Protocol frame has this pre-encoding structure:

```
┌──────┬──────┬──────────┬───────┐
│ type │ tag  │ payload  │ crc16 │
│  u8  │ u16  │  0..P    │  u16  │
└──────┴──────┴──────────┴───────┘
```

`P` is the maximum payload size for the largest message type, computed as `max_payload_bytes + 20` bytes (to accommodate the `RX` event's 20-byte metadata prefix plus the `bytes` field). `max_payload_bytes` is reported by the device in `GET_INFO` (Section 6.2) and reflects the chip's maximum over-the-air packet length.

The pre-encoding structure is then COBS-encoded (Section 2.4) and terminated with a single `0x00` byte on the wire:

```
┌────────────────────────────────────┬──────┐
│ COBS(type | tag | payload | crc16) │ 0x00 │
└────────────────────────────────────┴──────┘
```

All multi-byte integer fields are **little-endian**.

### 2.2 Fields

**`type`** (u8): Message type identifier. See Section 4.

**`tag`** (u16, little-endian):
- On host→device commands: a host-chosen correlation ID. MUST NOT be `0x0000`.
- On device→host command responses (`OK`, `ERR`, `TX_DONE`): echoes the tag of the originating command.
- On device→host asynchronous events (`RX`, asynchronous `ERR`): always `0x0000`.

Tag uniqueness: a tag is **outstanding** from the moment the host transmits the command until the host has received the final response for that tag (`OK`, `ERR`, or for `TX` the `TX_DONE`). Hosts MUST NOT reuse a tag that is currently outstanding. Once a tag is closed it may be reused freely. A monotonic counter (starting at 1, wrapping at `0xFFFF` and skipping `0x0000`) is the recommended implementation.

Tag = 0x0000 on a host→device command is a protocol violation. The device MUST treat such a frame as a framing error and discard it (emitting asynchronous `ERR(EFRAME)` per Section 2.5), because a response with tag 0x0000 would be indistinguishable from an asynchronous event. Hosts MUST never send tag 0x0000.

**`payload`** (variable length, 0 to `max_payload_bytes + 20` bytes):
- Structure defined per message type (Section 6).
- Length is implicit from the frame boundary: the payload runs from the byte after the tag to two bytes before the end of the decoded frame.

**`crc16`** (u16, little-endian):
- CRC-16/CCITT-FALSE, computed over `type || tag || payload` (pre-COBS bytes, not including the CRC itself).
- Polynomial: `0x1021`
- Initial value: `0xFFFF`
- No reflection (input or output)
- XOR out: `0x0000`
- Check value for ASCII `"123456789"`: `0x29B1`

This variant is also known as CRC-16/AUTOSAR or CRC-16/IBM-3740. It is NOT the same as CRC-16/KERMIT, CRC-16/XMODEM, or CRC-16/CCITT (which use different initial values or reflect their input).

### 2.3 Frame Size Bounds

- Minimum pre-COBS frame: 5 bytes (type + tag + CRC, empty payload).
- Maximum pre-COBS frame: `5 + max_payload_bytes + 20` bytes. For a device reporting `max_payload_bytes = 255`, this is 280 bytes.
- COBS adds at most `ceil(pre_cobs_length / 254) + 1` bytes of overhead.
- Plus one `0x00` delimiter byte.

Implementations SHOULD use buffer sizes of at least `max_payload_bytes + 32` bytes to accommodate all frames comfortably.

### 2.4 COBS Encoding

Frames are encoded with Consistent Overhead Byte Stuffing (COBS), using `0x00` as the framing byte. This guarantees that `0x00` never appears within an encoded frame, so a single `0x00` reliably marks frame boundaries.

**Algorithm**: The standard COBS algorithm (Cheshire & Baker, 1999). Reference implementation in Appendix A.

**Decoder resynchronization**: A receiver that opens the stream mid-frame, or that has lost bytes, discards received bytes until the next `0x00` and begins decoding at the byte following that `0x00`. No other synchronization mechanism is used.

**Overhead**: At most `ceil(N / 254) + 1` bytes for a pre-encoded frame of N bytes, plus the trailing `0x00` delimiter. For typical frames (under 254 bytes), overhead is 2 bytes.

### 2.5 Receiver Parsing Algorithm

On receipt of a byte stream:

1. Accumulate bytes until a `0x00` byte is received. That `0x00` is the frame delimiter.
2. COBS-decode the bytes that preceded the delimiter.
3. If the decoded length is less than 5 bytes, discard. Device MAY emit asynchronous `ERR(EFRAME)`.
4. Split the decoded bytes: `type` (1) || `tag` (2) || `payload` (variable) || `crc16` (2).
5. Compute CRC-16/CCITT-FALSE over `type || tag || payload`. If it does not match `crc16`, discard. Device MAY emit asynchronous `ERR(EFRAME)`.
6. Dispatch by `type`. Unknown `type` byte on the device side: emit `ERR(EUNKNOWN_CMD)` with the received tag. Unknown `type` on the host side: silently discard.

A partial frame at stream close (bytes received without a terminating `0x00`) is always discarded.

---

## 3. State Machine

The device has exactly two states: **UNCONFIGURED** and **CONFIGURED**.

```
                ┌──────────────────┐
                │   UNCONFIGURED   │  ← initial state on boot
                └──────────────────┘
                 │                ▲
   SET_CONFIG    │                │  inactivity timeout (1 s)
   result=       │                │  OR client disconnect
   APPLIED       ▼                │
                ┌──────────────────┐
                │    CONFIGURED    │
                └──────────────────┘
```

### 3.1 UNCONFIGURED

- Entered on boot, after inactivity timeout, or on final client disconnect.
- Radio is in standby (TX disabled, RX disabled, minimal current).
- TX queue is empty. RX ring is empty.
- Accepted commands: `PING`, `GET_INFO`, `SET_CONFIG`.
- Other commands (`TX`, `RX_START`, `RX_STOP`) return `ERR(ENOTCONFIGURED)`.

### 3.2 CONFIGURED

- Entered when `SET_CONFIG` returns `result = APPLIED`.
- Radio parameters match the most recent successful `SET_CONFIG` payload.
- All commands accepted.
- Radio is idle (no RX, no TX) unless `RX_START` has been issued or a TX is in flight.

### 3.3 Transitions

**UNCONFIGURED → CONFIGURED**: successful `SET_CONFIG` with `result = APPLIED`.

**CONFIGURED → UNCONFIGURED**:
- Inactivity timer expires (no frames received for 1000 ms), OR
- Final client disconnects at the transport layer.

Note: `SET_CONFIG` results of `ALREADY_MATCHED` or `LOCKED_MISMATCH` do NOT transition state. They are responses observing the current configuration.

**Within CONFIGURED**: `SET_CONFIG` may be issued again to change params (subject to the lock in multi-client, Section 13). See Section 3.5 for operations spanning this transition.

### 3.4 Liveness Timer

- Each client maintains an inactivity timer with a fixed period of **1000 ms**.
- The timer is **inert** until the first frame from that client has been received. A freshly-booted device with no activity does not time out. The timer starts when the first frame arrives and resets on every subsequent frame.
- The timer is reset whenever any byte stream activity results in a complete frame arriving, whether that frame parses successfully or not. Even a frame that fails CRC or COBS resets the timer — the bytes arrived, meaning the host is present.
- If the timer expires:
  1. All queued-but-not-started TXs issued by that client are discarded silently. No `TX_DONE` is emitted.
  2. Any TX on the air is allowed to complete (it cannot be cleanly aborted). No `TX_DONE` is emitted.
  3. In single-client: the device transitions to UNCONFIGURED. RX ring is cleared. Any continuous RX is aborted. The timer returns to the inert state until the next frame arrives.
  4. In multi-client: only that client's session state is cleaned up. See Section 13.

A client whose inactivity timer has fired is considered disconnected. Transport-level disconnect events (USB detach, BLE disconnect, TCP close) have identical effect to timer expiration.

### 3.5 Operations Spanning Reconfiguration

**`SET_CONFIG` while a TX is on the air**: the in-progress TX is allowed to physically complete. Its `TX_DONE` is emitted normally. Any queued-but-not-yet-started TXs are cancelled and emit `TX_DONE(result=CANCELLED)`. Continuous RX, if active, is aborted. RX ring is cleared. New configuration takes effect after the in-flight TX finishes.

**`SET_CONFIG` validation failure (`ERR(EPARAM)`, `ERR(ELENGTH)`, `ERR(EMODULATION)`)**: the chip is not touched. All parameters are validated before any SPI write to the radio. If validation fails, the radio and state machine are unchanged.

**`SET_CONFIG` with no lock conflict, applied successfully**: the radio is driven through its configuration sequence (which includes any required image calibration, ~3–15 ms on SX126x). The `OK` response is not emitted until the radio has completed configuration and is ready to accept TX / RX commands.

**Hardware failure during `SET_CONFIG`**: If the radio does not respond to SPI commands or returns unexpected status during configuration, the device returns `ERR(ERADIO)` synchronously with the command's tag. The radio is placed in the safest reachable state (typically standby) and the device transitions to UNCONFIGURED. No follow-up asynchronous `ERR` is emitted for the same fault.

**Disconnect during `SET_CONFIG`**: If the client disconnects while the radio is being reconfigured (the ~15 ms calibration window), the configuration completes to the chip (it is not interrupted), the `OK` is generated and then silently dropped, and normal disconnect handling proceeds: the device transitions to UNCONFIGURED and re-idles the radio. No partial-configuration state is possible.

---

## 4. Message Types

| Hex  | Direction | Name         | Brief                                     |
|------|-----------|--------------|-------------------------------------------|
| 0x01 | H→D       | PING         | Liveness                                  |
| 0x02 | H→D       | GET_INFO     | Query capabilities and identity           |
| 0x03 | H→D       | SET_CONFIG   | Configure the radio                       |
| 0x04 | H→D       | TX           | Transmit a packet                         |
| 0x05 | H→D       | RX_START     | Begin continuous reception                |
| 0x06 | H→D       | RX_STOP      | End continuous reception                  |
| 0x80 | D→H       | OK           | Positive response                         |
| 0x81 | D→H       | ERR          | Negative response or asynchronous fault   |
| 0xC0 | D→H       | RX           | Packet received                           |
| 0xC1 | D→H       | TX_DONE      | TX concluded (transmitted/busy/cancelled) |

All other `type` byte values are reserved. Devices MUST reject unknown command types (bytes with MSB = 0) with `ERR(EUNKNOWN_CMD)` echoing the received tag. Hosts MUST silently discard frames with unknown device→host type bytes (MSB = 1).

---

## 5. Common Data Types

### 5.1 Scalar Encodings

| Name | Width | Encoding                                 |
|------|-------|------------------------------------------|
| u8   | 1     | Unsigned byte                            |
| u16  | 2     | Unsigned, little-endian                  |
| u32  | 4     | Unsigned, little-endian                  |
| u64  | 8     | Unsigned, little-endian                  |
| i8   | 1     | Signed byte, two's complement            |
| i16  | 2     | Signed, little-endian, two's complement  |
| i32  | 4     | Signed, little-endian, two's complement  |

### 5.2 Physical-Unit Encodings

| Quantity         | Encoding                                                 |
|------------------|----------------------------------------------------------|
| Frequency        | `u32`, hertz                                             |
| TX power         | `i8`, dBm                                                |
| RSSI             | `i16`, tenths of dBm (e.g., −1234 = −123.4 dBm)          |
| SNR              | `i16`, tenths of dB (e.g., −45 = −4.5 dB)                |
| Frequency error  | `i32`, hertz                                             |
| Time interval    | `u32`, microseconds                                      |
| Monotonic time   | `u64`, microseconds since device boot (does not wrap)    |

### 5.3 Variable-Length Fields

Fields whose length is not explicitly given in a message's payload definition extend from their stated start offset to the end of the frame's payload. Their length is implicit from the total frame length.

Each message type has at most two variable-length fields, and they always appear last. When two appear together (e.g., the UIDs in `GET_INFO` response), explicit length prefixes disambiguate them.

---

## 6. Message Reference

### 6.1 PING (0x01, H→D)

**Purpose**: Keepalive. Sent by the host when it has no other work to do, to prevent the device's inactivity timer from expiring. May also be used to measure round-trip latency.

**Payload**: Empty (0 bytes).

**Response**: `OK` with empty payload, tag echoed.

**Valid in**: any state (UNCONFIGURED or CONFIGURED).

**Errors**: none specific.

---

### 6.2 GET_INFO (0x02, H→D)

**Purpose**: Query the device's identity, firmware version, radio type, capabilities, and queue capacities. Optional — the host is not required to call it. Typically called once, after connecting.

**Payload**: Empty.

**Response**: `OK` with the following payload, tag echoed:

```
Offset  Size  Field                Description
 0      1     proto_major          u8, protocol major version (1 for this spec)
 1      1     proto_minor          u8, protocol minor version (0 for this spec)
 2      1     fw_major             u8, firmware major
 3      1     fw_minor             u8, firmware minor
 4      1     fw_patch             u8, firmware patch
 5      2     radio_chip_id        u16, see Section 8
 7      8     capability_bitmap    u64, see Section 9
15      2     supported_sf_bitmap  u16, bit N = LoRa SF N is supported (N in 0..15)
17      2     supported_bw_bitmap  u16, bit N = LoRa BW enum value N is supported (N in 0..15)
19      2     max_payload_bytes    u16, max over-the-air payload
21      2     rx_queue_capacity    u16, RX ring depth
23      2     tx_queue_capacity    u16, TX queue depth
25      4     freq_min_hz          u32, minimum carrier frequency the radio supports
29      4     freq_max_hz          u32, maximum carrier frequency the radio supports
33      1     tx_power_min_dbm     i8, minimum TX power (most negative)
34      1     tx_power_max_dbm     i8, maximum TX power
35      1     mcu_uid_len          u8, 0 to 32
36      N     mcu_uid              N = mcu_uid_len
36+N    1     radio_uid_len        u8, 0 to 16
37+N    M     radio_uid            M = radio_uid_len
```

Total payload size: `37 + mcu_uid_len + radio_uid_len` bytes.

**Valid in**: any state.

**Errors**: none specific.

**Notes**:
- `radio_uid_len` MAY be 0 if the radio has no accessible unique identifier.
- `mcu_uid` and `radio_uid` are opaque byte strings; hosts SHOULD treat them as unique identifiers for the silicon and not interpret their internal structure.
- `capability_bitmap` tells the host which modulations and features this device supports before it attempts `SET_CONFIG`.
- `supported_sf_bitmap` and `supported_bw_bitmap` describe LoRa-specific capabilities. Each bit N corresponds to value N of the respective enum (SF index for SF, BW enum value for BW). Hosts SHOULD consult these before issuing `SET_CONFIG` to avoid trial-and-error. Rationale: some chips have gaps (e.g., LLCC68 supports SF5–SF11 but not SF12; SX127x supports SF6–SF12 but not SF5), which a min/max pair cannot express.
- Both bitmaps are meaningful only when `capability_bitmap` bit 0 (LoRa) is set. On non-LoRa-capable devices they MAY be zero.
- `rx_queue_capacity` and `tx_queue_capacity` help the host pipeline intelligently without overrunning the device.
- `freq_min_hz` / `freq_max_hz` bracket the RF front-end's supported range for this specific board. A chip's silicon may cover a wider range than the matching network permits, so these values reflect the board's effective tuning window.
- `tx_power_min_dbm` / `tx_power_max_dbm` bracket the chip's settable TX power register range. The host SHOULD validate its `tx_power_dbm` against these bounds before calling `SET_CONFIG`; values outside the range yield `ERR(EPARAM)`.
- All `GET_INFO` fields are stable for the lifetime of a session. The device MUST NOT change them after boot. Hosts MAY cache the response for the duration of the connection.
- If the firmware cannot identify its radio (e.g., SPI probe failed), `radio_chip_id` is `0x0000` and the host SHOULD abort — the device will not function.

---

### 6.3 SET_CONFIG (0x03, H→D)

**Purpose**: Configure the radio for a given modulation and parameter set. This is the only command that can transition the device from UNCONFIGURED to CONFIGURED. It may also be issued in CONFIGURED to change parameters.

**Payload**:

```
Offset  Size  Field           Description
 0      1     modulation_id   u8, see Section 10
 1      N     params          Modulation-specific struct (Section 10)
```

The length of `params` is determined entirely by `modulation_id` (Section 10). Sending a payload whose total length does not match `1 + expected_param_size` yields `ERR(ELENGTH)`.

**Response (on success)**: `OK` with the following payload, tag echoed:

```
Offset  Size  Field              Description
 0      1     result             u8: 0=APPLIED, 1=ALREADY_MATCHED, 2=LOCKED_MISMATCH
 1      1     owner              u8: 0=NONE, 1=MINE, 2=OTHER
 2      1     current_modulation u8, modulation currently in effect on the radio
 3      N     current_params    Parameters currently in effect
```

`current_modulation` and `current_params` always reflect what is actually programmed into the radio at the moment the response is constructed, regardless of whether this call caused a change. This gives the host a consistent ground truth in every case.

**Result values**:
- **APPLIED (0)**: The request was accepted. The radio is now configured as requested. The calling client owns the lock (single-client: always `MINE`).
- **ALREADY_MATCHED (1)**: Another client holds the lock, but the active configuration byte-for-byte matches the requested one. No change was made. The calling client may proceed to use the radio cooperatively but does not own the lock.
- **LOCKED_MISMATCH (2)**: Another client holds the lock and the active configuration differs. No change was made. The calling client may inspect `current_params` to diff.

**Owner values**:
- **NONE (0)**: No client currently holds the lock.
- **MINE (1)**: The calling client holds the lock.
- **OTHER (2)**: A different client holds the lock.

**Valid in**: any state.

**Errors** (all synchronous, with tag echoed):
- `ERR(EMODULATION)`: `modulation_id` is not supported on this chip. Check the capability bitmap.
- `ERR(ELENGTH)`: payload length does not match the expected size for `modulation_id`.
- `ERR(EPARAM)`: a parameter value is out of range (e.g., frequency outside chip's supported range, SF out of range, reserved field nonzero).

**Atomicity**: All parameters are validated before any SPI write is issued to the radio. On validation failure the radio is untouched; the device returns `ERR` and remains in its prior state (UNCONFIGURED or CONFIGURED with prior params). Validation cannot leave the radio in an intermediate state.

**Side effects on `APPLIED`**:
- Any in-progress TX is allowed to finish; its `TX_DONE` is emitted normally.
- All queued (not yet started) TXs are cancelled with `TX_DONE(result=CANCELLED)`.
- Any continuous RX is stopped.
- RX ring is cleared.
- Radio is reconfigured, calibrated if required, and left in idle. The `OK` response is not emitted until this sequence completes.

**Notes**:
- **Auto-LDRO**: For LoRa, the device automatically enables Low Data Rate Optimization when `(2^SF) / BW_Hz > 16 ms` (the Semtech-recommended threshold). The host does not specify LDRO and is not informed of its state.
- Reconfiguring to identical parameters is a valid call. In single-client mode, the response is always `result = APPLIED, owner = MINE`. No short-circuit is performed; the radio is genuinely re-initialized.
- In multi-client mode, see Section 13.2 for lock behavior.

---

### 6.4 TX (0x04, H→D)

**Purpose**: Transmit a single packet.

**Payload**:

```
Offset  Size  Field       Description
 0      1     flags       u8, see below
 1      N     bytes       Packet bytes, 1 ≤ N ≤ max_payload_bytes
```

**Flags** (bit 0 = LSB):
- **Bit 0 — `skip_cad`**: If 1, skip the default CAD-before-TX and transmit immediately. If 0, the device performs a CAD cycle first and transmits only if the channel is clear (emitting `TX_DONE(CHANNEL_BUSY)` otherwise). Default behavior (bit 0 = 0) is the recommended path.
- **Bits 1–7**: Reserved. MUST be 0. Device rejects frames with any reserved bit set via `ERR(EPARAM)`.

**Response**: `OK` with empty payload, tag echoed. This response indicates the command was accepted and enqueued. It does NOT indicate the packet was transmitted. The transmission outcome is delivered later as a `TX_DONE` event with the same tag.

**Asynchronous completion**: exactly one `TX_DONE` event with matching tag (Section 6.10), unless the command failed synchronously (received `ERR`, no `TX_DONE` follows) or the client disconnected before transmission (no `TX_DONE` emitted, since the client is gone).

**Valid in**: CONFIGURED only.

**Errors**:
- `ERR(ENOTCONFIGURED)`: device is UNCONFIGURED.
- `ERR(ELENGTH)`: N = 0 (zero-byte packets are not permitted), N > `max_payload_bytes`, or N exceeds the current modulation's per-mode maximum (which may be less than `max_payload_bytes`).
- `ERR(EPARAM)`: a reserved flag bit is set.
- `ERR(EBUSY)`: internal TX queue is full (already holds `tx_queue_capacity` pending TXs). Host should wait for an outstanding `TX_DONE` before retrying with a new tag.

**Max payload note**: `max_payload_bytes` from `GET_INFO` is the chip-wide maximum. Individual modulations may have stricter limits (e.g., LR-FHSS payloads are ≤ ~100 bytes depending on configuration). A packet larger than the current modulation supports is rejected with `ERR(ELENGTH)`, even if it fits within `max_payload_bytes`.

**CAD-default behavior on non-CAD modulations**: On modulations that do not support CAD (FSK, GFSK, FLRC, etc.), the `skip_cad` flag is silently ignored and the TX fires immediately. Host code does not change between LoRa and non-LoRa modes.

**RX interruption by TX**: On half-duplex chips (all Semtech transceivers except when capability bit 22 is set), the radio briefly leaves RX to transmit and returns to RX afterward. A packet that was mid-reception during the transmission window is lost. On full-duplex chips, TX does not interrupt RX.

**CAD timing**: For LoRa, the CAD duration depends on SF and BW. At SF7/BW125 it is ~4 ms; at SF12/BW125 it is ~130 ms. Hosts that need tight TX latency should either use a lower SF or set `skip_cad = 1`.

---

### 6.5 RX_START (0x05, H→D)

**Purpose**: Place the radio in continuous receive mode. Every received packet (including ones with bad CRC) produces an `RX` event.

**Payload**: Empty.

**Response**: `OK` with empty payload, tag echoed.

**Valid in**: CONFIGURED only.

**Errors**:
- `ERR(ENOTCONFIGURED)`: device is UNCONFIGURED.
- `ERR(EMODULATION)`: the currently-configured modulation does not support reception (e.g., LR-FHSS is transmit-only on all current Semtech silicon).

**Notes**:
- Calling `RX_START` when already in continuous RX is a no-op. Response is `OK`.
- A `TX` interrupts RX, transmits, and the radio automatically returns to continuous RX afterward without a new `RX_START`. (On full-duplex chips, the interruption does not occur.)
- `SET_CONFIG` aborts continuous RX. Host must re-issue `RX_START` after reconfiguration if it still wants to receive.

---

### 6.6 RX_STOP (0x06, H→D)

**Purpose**: End continuous reception; radio returns to idle.

**Payload**: Empty.

**Response**: `OK` with empty payload, tag echoed.

**Valid in**: CONFIGURED only.

**Errors**:
- `ERR(ENOTCONFIGURED)`: device is UNCONFIGURED.

**Notes**:
- Calling `RX_STOP` when not in continuous RX is a no-op. Response is `OK`.
- RX events already in the ring at the time of `RX_STOP` remain queued; the host continues to receive them as normal `RX` frames until the ring drains.

---

### 6.7 OK (0x80, D→H)

**Purpose**: Positive response to a host command.

**Tag**: Echoes the originating command's tag. Never `0x0000`.

**Payload**: Structure depends on originating command:
- `PING`, `TX`, `RX_START`, `RX_STOP`: empty.
- `GET_INFO`: info struct (Section 6.2).
- `SET_CONFIG`: result / owner / current-config struct (Section 6.3).

**Notes**: To correctly interpret an `OK` payload, the host must know which command issued the echoed tag. Outstanding-tag bookkeeping is mandatory host-side.

---

### 6.8 ERR (0x81, D→H)

**Purpose**: Negative response to a command, OR asynchronous radio / protocol fault.

**Tag**:
- Non-zero → synchronous error, tag echoes the originating command. The command was not carried out and no compensating event will follow.
- Zero → asynchronous error, not tied to any specific command. Fanned out to all connected clients in multi-client mode.

**Payload**:

```
Offset  Size  Field   Description
 0      2     code    u16, error code (Section 7)
```

---

### 6.9 RX (0xC0, D→H)

**Purpose**: Unsolicited event indicating a packet was received over the air (or, in multi-client mode, that another client transmitted and this is the loopback).

**Tag**: Always `0x0000`.

**Payload**:

```
Offset  Size  Field             Description
 0      2     rssi              i16, tenths of dBm
 2      2     snr               i16, tenths of dB
 4      4     freq_err          i32, hertz, estimated LO offset
 8      8     timestamp_us      u64, µs since boot, captured at RxDone IRQ (packet end)
16      1     crc_valid         u8: 0=CRC failed, 1=CRC passed or CRC not configured
17      2     packets_dropped   u16, packets dropped since previous delivered RX
19      1     origin            u8: 0=over-the-air, 1=device-local loopback
20      N     bytes             Packet payload, N determined by frame length
```

**Field semantics**:
- `timestamp_us` is captured the instant the radio asserts RxDone (the end-of-packet IRQ).
- `crc_valid = 0` packets are delivered to the host anyway, because they may still be of interest (debugging, partial capture). The bytes may be corrupt.
- When `crc_valid = 0`, `snr` and `freq_err` may be meaningless.
- `packets_dropped` is cleared to 0 on every delivered `RX` event. It represents losses since the last successful delivery.
- `origin = 1` is only possible in multi-client mode (Section 13). In single-client it is always 0.
- `bytes` has length `N ≥ 0` and always satisfies `N ≤ max_payload_bytes`. If the radio reports a length field larger than `max_payload_bytes` (possible on a corrupted PHY header), the device MUST truncate to `max_payload_bytes` and set `crc_valid = 0`. Hosts can rely on `N ≤ max_payload_bytes` unconditionally.

---

### 6.10 TX_DONE (0xC1, D→H)

**Purpose**: Asynchronous completion of a previously-issued `TX`.

**Tag**: Echoes the tag of the originating `TX`.

**Payload**:

```
Offset  Size  Field        Description
 0      1     result       u8: 0=TRANSMITTED, 1=CHANNEL_BUSY, 2=CANCELLED
 1      4     airtime_us   u32, on-air duration in microseconds (0 for non-TRANSMITTED)
```

**Result values**:
- **TRANSMITTED (0)**: Packet went on the air. `airtime_us` is the measured or computed time-on-air.
- **CHANNEL_BUSY (1)**: CAD detected activity; TX was aborted before transmission. `airtime_us = 0` (CAD time itself is not counted). This result is only possible when `skip_cad = 0` was set in the originating TX **and** the current modulation supports CAD (LoRa only). Any TX with `skip_cad = 1`, or any TX on a non-CAD modulation, will never produce CHANNEL_BUSY.
- **CANCELLED (2)**: TX was discarded because `SET_CONFIG` or disconnect occurred before the packet reached the air. `airtime_us = 0`.

**Notes**:
- Exactly one `TX_DONE` is emitted per `TX` that received an `OK`. A `TX` that received `ERR` produces no `TX_DONE`.
- On client disconnect, `TX_DONE` for cancelled TXs is not emitted — the client is gone.
- On `CHANNEL_BUSY`, hosts should apply randomized backoff and retry with a **new tag**. The prior tag's state is closed and the same tag cannot be reused while any earlier-issued outstanding tag is still open.

---

## 7. Error Codes

All error codes are `u16` little-endian. The tag on the enclosing `ERR` frame distinguishes context: non-zero tag = synchronous error tied to a specific command; tag 0x0000 = asynchronous error not tied to a command. The two tables below group codes by the context in which they are *typically* emitted, but the device MAY emit any code in either context when it fits semantically (e.g., `ERR(ERADIO)` is listed as asynchronous but is emitted synchronously when a hardware fault occurs during processing of a specific command, such as SPI failure during `SET_CONFIG`).

### 7.1 Synchronous (tag ≠ 0), typical

| Code   | Name            | Meaning                                                       |
|--------|-----------------|---------------------------------------------------------------|
| 0x0001 | EPARAM          | A parameter value is out of range or invalid.                 |
| 0x0002 | ELENGTH         | Payload length is wrong for the command or modulation.        |
| 0x0003 | ENOTCONFIGURED  | Command requires CONFIGURED; device is UNCONFIGURED.          |
| 0x0004 | EMODULATION     | Requested modulation is not supported by this chip.           |
| 0x0005 | EUNKNOWN_CMD    | Unknown command type byte.                                    |
| 0x0006 | EBUSY           | Transient: TX queue full. Host should wait and retry.         |

Codes 0x0000–0x00FF are reserved for synchronous errors.

### 7.2 Asynchronous (tag = 0), typical

| Code   | Name      | Meaning                                                              |
|--------|-----------|----------------------------------------------------------------------|
| 0x0101 | ERADIO    | Radio SPI error or unexpected hardware state.                        |
| 0x0102 | EFRAME    | Inbound frame had bad CRC, bad COBS, or wrong length.                |
| 0x0103 | EINTERNAL | Firmware encountered an unexpected internal condition.               |

Codes 0x0100–0x01FF are reserved for asynchronous errors. Codes 0x0200 and above are reserved for future extensions.

**Impact of asynchronous errors on pending commands**: An `EFRAME` on a frame that was *supposed* to be a response leaves the host's corresponding outstanding tag open indefinitely from the device's perspective. Hosts MUST implement their own command timeouts (Section 14.2) to avoid tag exhaustion or indefinite waiting.

---

## 8. Radio Chip ID Enum

| Value   | Chip   | Family              |
|---------|--------|---------------------|
| 0x0000  | Unknown| —                   |
| 0x0001  | SX1261 | Gen 2 sub-GHz       |
| 0x0002  | SX1262 | Gen 2 sub-GHz       |
| 0x0003  | SX1268 | Gen 2 sub-GHz       |
| 0x0004  | LLCC68 | Gen 2 sub-GHz       |
| 0x0010  | SX1272 | Gen 1 sub-GHz       |
| 0x0011  | SX1276 | Gen 1 sub-GHz       |
| 0x0012  | SX1277 | Gen 1 sub-GHz       |
| 0x0013  | SX1278 | Gen 1 sub-GHz       |
| 0x0014  | SX1279 | Gen 1 sub-GHz       |
| 0x0020  | SX1280 | Gen 2 2.4 GHz       |
| 0x0021  | SX1281 | Gen 2 2.4 GHz       |
| 0x0030  | LR1110 | Gen 3 multi-band    |
| 0x0031  | LR1120 | Gen 3 multi-band    |
| 0x0032  | LR1121 | Gen 3 multi-band    |
| 0x0040  | LR2021 | Gen 4 multi-band    |

Values not listed here are reserved. Future chips will be assigned in the next available slot of their family's range.

---

## 9. Capability Bitmap

The `capability_bitmap` field in `GET_INFO` is a `u64`. Bits are numbered from LSB. Undefined bits MUST be 0 in v1.0. Hosts MUST ignore undefined bits.

### 9.1 Modulations (bits 0–15)

| Bit | Modulation       |
|-----|------------------|
| 0   | LoRa             |
| 1   | FSK              |
| 2   | GFSK             |
| 3   | LR-FHSS          |
| 4   | FLRC             |
| 5   | MSK              |
| 6   | GMSK             |
| 7   | BLE-compatible   |
| 8–15| Reserved         |

A bit set here means the device's `SET_CONFIG` accepts the corresponding `modulation_id`.

Multiple modulation bits being set does NOT imply the device can operate in multiple modulations simultaneously. The radio is configured for exactly one modulation at a time (whichever was most recently applied via `SET_CONFIG`). To switch modulations, issue another `SET_CONFIG`.

### 9.2 Radio Features (bits 16–31)

| Bit | Feature                                                              |
|-----|----------------------------------------------------------------------|
| 16  | CAD-before-TX available (firmware performs CAD in default TX path)   |
| 17  | IQ inversion supported in LoRa mode                                  |
| 18  | Ranging / time-of-flight distance measurement                        |
| 19  | GNSS scanning                                                        |
| 20  | WiFi MAC scanning                                                    |
| 21  | Spectral scan                                                        |
| 22  | Full-duplex (simultaneous TX + RX)                                   |
| 23–31| Reserved                                                            |

If bit 16 is 0, the device does not perform CAD before TX even for LoRa mode; the `skip_cad` flag has no effect.

### 9.3 Protocol Features (bits 32–47)

| Bit | Feature              |
|-----|----------------------|
| 32  | Multi-client support |
| 33–47| Reserved            |

### 9.4 Reserved (bits 48–63)

All reserved for future use.

---

## 10. Modulation Parameters

The `modulation_id` byte in `SET_CONFIG` selects one of the following parameter structs. Any value not listed here is reserved and yields `ERR(EMODULATION)` (if the chip conceptually supports the modulation but the ID is not yet defined) or `ERR(EMODULATION)` (if the chip does not support it at all).

| modulation_id | Modulation | Fixed struct size     |
|---------------|------------|------------------------|
| 0x01          | LoRa       | 15 bytes               |
| 0x02          | FSK / GFSK | 16 + `sync_word_len`   |
| 0x03          | LR-FHSS    | 10 bytes               |
| 0x04          | FLRC       | 13 bytes               |

### 10.1 LoRa (modulation_id = 0x01, 15 bytes)

```
Offset  Size  Field           Range / notes
 0      4     freq_hz         u32, chip-supported range (EPARAM if outside)
 4      1     sf              u8, 5–12 (chip-dependent: SX127x is 6–12)
 5      1     bw              u8, bandwidth enum (see below)
 6      1     cr              u8, coding-rate enum (see below)
 7      2     preamble_len    u16, in symbols (typical min 6)
 9      2     sync_word       u16 (SX127x: uses low byte only; high byte MUST be 0)
11      1     tx_power_dbm    i8, chip-dependent range
12      1     header_mode     u8, 0=explicit, 1=implicit
13      1     payload_crc     u8, 0=disabled, 1=enabled
14      1     iq_invert       u8, 0=normal, 1=inverted
```

**Bandwidth enum** (the small integer on the wire, not the kHz value):

| Value | Bandwidth  | Applicable chip families        |
|-------|------------|---------------------------------|
| 0     | 7.81 kHz   | SX126x, SX127x, LR11xx, LR2021  |
| 1     | 10.42 kHz  | SX126x, SX127x, LR11xx, LR2021  |
| 2     | 15.63 kHz  | SX126x, SX127x, LR11xx, LR2021  |
| 3     | 20.83 kHz  | SX126x, SX127x, LR11xx, LR2021  |
| 4     | 31.25 kHz  | SX126x, SX127x, LR11xx, LR2021  |
| 5     | 41.67 kHz  | SX126x, SX127x, LR11xx, LR2021  |
| 6     | 62.5 kHz   | SX126x, SX127x, LR11xx, LR2021  |
| 7     | 125 kHz    | all                             |
| 8     | 250 kHz    | all                             |
| 9     | 500 kHz    | all                             |
| 10    | 200 kHz    | SX128x                          |
| 11    | 400 kHz    | SX128x                          |
| 12    | 800 kHz    | SX128x                          |
| 13    | 1600 kHz   | SX128x                          |

A value not supported by the current radio yields `ERR(EPARAM)`. Hosts SHOULD consult `radio_chip_id` and select from the applicable set.

**Coding-rate enum**:

| Value | Coding rate |
|-------|-------------|
| 0     | 4/5         |
| 1     | 4/6         |
| 2     | 4/7         |
| 3     | 4/8         |

Any value outside these enumerations produces `ERR(EPARAM)`.

### 10.2 FSK / GFSK (modulation_id = 0x02, variable size)

```
Offset  Size  Field           Description
 0      4     freq_hz         u32, chip-supported range
 4      4     bitrate_bps     u32, bits per second
 8      4     freq_dev_hz     u32, FSK frequency deviation
12      1     rx_bw           u8, chip-specific RX bandwidth enum
13      2     preamble_len    u16, in bits
15      1     sync_word_len   u8, 0–8
16      N     sync_word       N = sync_word_len bytes
```

Total struct size: `16 + sync_word_len` bytes.

**Sync word byte order**: The byte at offset 16 of the payload is transmitted first on the air; subsequent bytes follow in order. Within each byte, the MSB is transmitted first (standard FSK/GFSK convention).

GFSK is selected by the firmware when the chip supports Gaussian filtering at the requested bitrate; the wire protocol does not distinguish it from FSK beyond the capability bitmap.

### 10.3 LR-FHSS (modulation_id = 0x03, 10 bytes)

```
Offset  Size  Field           Description
 0      4     freq_hz         u32, center frequency
 4      1     bw_enum         u8, LR-FHSS occupied-bandwidth enum (see below)
 5      1     cr_enum         u8, LR-FHSS coding-rate enum (see below)
 6      1     grid            u8, 0=25.39 kHz grid, 1=3.9 kHz grid
 7      1     hopping         u8, 0=disabled, 1=enabled
 8      1     tx_power_dbm    i8
 9      1     reserved        u8, MUST be 0 (EPARAM otherwise)
```

**LR-FHSS bandwidth enum**:

| Value | Occupied BW |
|-------|-------------|
| 0     | 39.06 kHz   |
| 1     | 85.94 kHz   |
| 2     | 136.72 kHz  |
| 3     | 183.59 kHz  |
| 4     | 335.94 kHz  |
| 5     | 386.72 kHz  |
| 6     | 722.66 kHz  |
| 7     | 1523.44 kHz |

**LR-FHSS coding-rate enum**:

| Value | Coding rate |
|-------|-------------|
| 0     | 5/6         |
| 1     | 2/3         |
| 2     | 1/2         |
| 3     | 1/3         |

### 10.4 FLRC (modulation_id = 0x04, 13 bytes)

```
Offset  Size  Field           Description
 0      4     freq_hz         u32, carrier frequency
 4      1     bitrate_enum    u8, bitrate enum (see below)
 5      1     cr_enum         u8, FLRC coding-rate (0=1/2, 1=3/4, 2=1/1)
 6      1     bt_enum         u8, Gaussian BT product (0=off, 1=0.5, 2=1.0)
 7      1     preamble_len    u8, preamble length enum (see below)
 8      4     sync_word       u32, 32-bit sync word (transmitted MSB-first on air)
12      1     tx_power_dbm    i8
```

**FLRC bitrate enum**:

| Value | Bitrate     |
|-------|-------------|
| 0     | 2600 kbps   |
| 1     | 2080 kbps   |
| 2     | 1300 kbps   |
| 3     | 1040 kbps   |
| 4     | 650 kbps    |
| 5     | 520 kbps    |
| 6     | 325 kbps    |
| 7     | 260 kbps    |

**FLRC preamble length enum**:

| Value | Preamble bits |
|-------|---------------|
| 0     | 8             |
| 1     | 12            |
| 2     | 16            |
| 3     | 20            |
| 4     | 24            |
| 5     | 28            |
| 6     | 32            |

Any enum value outside these tables yields `ERR(EPARAM)`.

FLRC specifics vary between SX128x and LR2021; the firmware maps chip-specific register values to these byte values. Values that don't correspond to a supported setting on the current chip produce `ERR(EPARAM)`.

---

## 11. Ordering and Interleaving

These invariants define what hosts and devices may rely on.

**Command responses are monotonic per session**:
- If the host sends command A (tag = T_A) before command B (tag = T_B), then the device's response to A (`OK` or `ERR` with tag T_A) is emitted before the response to B (`OK` or `ERR` with tag T_B).
- This does NOT apply across clients in multi-client mode — each client sees its own command responses in order, but responses across clients may interleave arbitrarily.

**TX_DONE order matches TX order**:
- If TX with tag T_1 was accepted before TX with tag T_2 (same client), then `TX_DONE(T_1)` is emitted before `TX_DONE(T_2)`.
- This follows from the serial nature of the TX queue.

**RX and async events are unordered relative to responses**:
- `RX` events and asynchronous `ERR` events (tag = 0) may appear at any point in the device→host stream, interleaved with command responses.
- A host reading the stream cannot assume that a response to a pending command will arrive before an `RX` event, even if the command was issued first.

**Exactly-once semantics**:
- Every command that received `OK` or `ERR` is concluded (plus the additional `TX_DONE` for `TX` that got `OK`).
- No duplicated responses, no lost responses (absent transport-level failure).

---

## 12. Flow Control and Backpressure

### 12.1 Transport-Level Flow Control

DongLoRa Protocol relies on the underlying byte-stream transport to provide flow control:
- **USB CDC-ACM / CDC-UART bridges**: bulk endpoints provide implicit flow control via NAKs when the receiver is not ready.
- **Plain UART**: no flow control (host SHOULD use a reasonable bitrate like 921600 bps and respect device processing time).
- **BLE**: L2CAP / GATT credit-based flow control.
- **TCP**: window-based flow control.

If the device's internal receive buffer is full, it MAY stall the transport (stop ACKing / NAKing bytes) until buffer space is freed.

### 12.2 Device-Side Queue Limits

- **TX queue**: depth = `tx_queue_capacity` (from `GET_INFO`). Additional TX attempts return `ERR(EBUSY)`. Host should wait for an outstanding `TX_DONE` before retrying.
- **RX ring**: depth = `rx_queue_capacity`. On overflow, oldest-first drop policy applies; the drop count is delivered in the next `RX` event's `packets_dropped` field.

### 12.3 Host-Side Expectations

Hosts SHOULD:
- Size their RX decoder buffer to at least `max_payload_bytes + 32` bytes.
- Not pipeline more commands than `tx_queue_capacity` (for TX) without waiting for responses.
- Drain the device→host stream continuously. A host that stops reading will eventually stall the device.

---

## 13. Multi-Client Operation (Future)

This section describes behavior on devices advertising capability bit 32 (multi-client). Single-client USB deployments may skip this section but note that multi-client-aware fields (`result`, `owner`, `origin`) appear on both and are defined consistently.

### 13.1 Per-Client Liveness

Each client has an independent 1000 ms inactivity timer. A client whose timer expires is disconnected; other clients are unaffected.

### 13.2 Configuration Lock

- The first `SET_CONFIG` from any client, while no lock is held, acquires the lock.
- The locking client may issue further `SET_CONFIG` commands freely; they always apply and return `result=APPLIED, owner=MINE`.
- A non-locking client's `SET_CONFIG`:
  - If requested params match active params byte-for-byte: `OK(result=ALREADY_MATCHED, owner=OTHER)`. No change.
  - Otherwise: `OK(result=LOCKED_MISMATCH, owner=OTHER)`. No change.
- The lock is released when the locking client disconnects (transport-level OR liveness timeout). If other clients are still connected, the device remains in CONFIGURED with the current params until the next successful `SET_CONFIG` from any client (which becomes the new locker). If no clients remain, the device transitions to UNCONFIGURED.

### 13.3 RX Distribution

`RX` events and asynchronous `ERR` events (tag = 0) are fanned out to every connected client.

### 13.4 TX Distribution and Cross-Client Loopback

- Each client's TXs go into a single global FIFO.
- Each client's `TX_DONE` is delivered only to the originating client.
- Each successful on-air TX (`TX_DONE(result=TRANSMITTED)`) is additionally delivered to all **other** connected clients as an `RX` event with:
  - `rssi`, `snr`, `freq_err` = 0
  - `timestamp_us` = captured at TxDone IRQ
  - `crc_valid` = 1
  - `packets_dropped` = 0
  - `origin` = 1
  - `bytes` = the transmitted packet

TXs that completed with `CHANNEL_BUSY` or `CANCELLED` did not go on the air and do NOT generate loopback events.

### 13.5 Single-Client Compatibility

In single-client mode:
- `owner` is always `NONE` (before `SET_CONFIG`) or `MINE` (after).
- `result` is always `APPLIED` for successful `SET_CONFIG`; `ALREADY_MATCHED` and `LOCKED_MISMATCH` never occur.
- `origin` on `RX` is always 0.

### 13.6 External Multiplexer Pattern

DongLoRa Protocol intentionally does not carry a client identifier on the wire. When multi-client behavior is needed without firmware-side support, an external software multiplexer can provide it:

- The multiplexer holds one single-client DongLoRa Protocol connection to the dongle (the dongle's sole client).
- Upstream, the multiplexer accepts multiple connections from downstream applications.
- For each frame flowing upward from the dongle, the multiplexer dispatches to the owning upstream connection based on its own internal `device_tag → (upstream_connection, upstream_tag)` mapping; for each frame flowing downward, the multiplexer rewrites the upstream-chosen tag into a fresh dongle-side tag and records the mapping.
- If the multiplexer wants to expose multi-client semantics to its upstream clients (the `result` / `owner` / `origin` fields defined in this section), it implements the locking, echo, and fan-out logic itself — the dongle is unaware.

This is identical to the work firmware would do in Section 13.1–13.4, just implemented one layer up. When firmware later gains native multi-client support (capability bit 32), upstream applications can migrate from talking to the multiplexer to talking to the firmware directly without changing their DongLoRa Protocol implementation: the wire format is unchanged, and the transport connection itself serves as the client identity. See Section 13.7.

### 13.7 Client Identity is the Transport Connection

DongLoRa Protocol does not encode a client identifier in any frame. Instead, each transport-level connection (one USB CDC-ACM session, one BLE GATT link, one TCP socket) IS the client identity from the firmware's perspective. Firmware with native multi-client support (capability bit 32) accepts multiple simultaneous transport connections and maintains per-connection state internally: which client owns the config lock, which client enqueued each pending TX, which connection to deliver each RX event on.

Consequences:
- No HELO or handshake is needed. A client opens its transport and immediately begins sending DongLoRa Protocol frames.
- The wire format is identical between single-client and multi-client firmware. A host written against single-client DongLoRa Protocol works unchanged against multi-client firmware; fields whose values don't vary for single-client (`owner`, `result = ALREADY_MATCHED/LOCKED_MISMATCH`, `origin = 1`) simply don't occur for that host.
- Firmware implementing multi-client pays the complexity of per-connection bookkeeping; single-client firmware pays nothing. Neither is on the wire.

---

## 14. Host Behavior Requirements

### 14.1 MUST

- Send at least one frame every 1000 ms while connected, to prevent the device's inactivity timer from expiring. `PING` exists for this purpose.
- Never use tag `0x0000` on outbound commands.
- Track outstanding tags; do not reuse a tag until its final response has been received (`OK`, `ERR`, or `TX_DONE` as applicable).
- Validate the CRC on every received frame; discard frames with mismatched CRC.
- Treat COBS decoding failures as framing errors; resynchronize on the next `0x00`.
- Set all reserved flag bits and reserved payload bytes to 0.

### 14.2 SHOULD

- Send keepalives at 500 ms intervals or better (2× margin on the 1000 ms device timeout).
- Use a monotonic tag counter starting at 1, wrapping at 0xFFFF and skipping 0x0000.
- Call `GET_INFO` once at connection to learn `max_payload_bytes`, `capability_bitmap`, queue capacities, frequency range, TX power range, and identity.
- Validate `freq_hz` and `tx_power_dbm` locally against `GET_INFO`-reported ranges before issuing `SET_CONFIG`.
- Apply a per-command timeout to detect hung or missing responses. A reasonable default: 2000 ms for commands that do not involve airtime; for `TX`, 2000 ms + the computed packet airtime + maximum expected CAD duration.
- On `TX_DONE(CHANNEL_BUSY)`, apply randomized exponential backoff before retrying with a new tag.
- On `RX(packets_dropped > 0)`, log or surface the loss.
- On receiving an unexpected `ERR(ENOTCONFIGURED)` in response to a command that was issued while the host believed the device to be CONFIGURED, treat this as a lost-session signal: the device has timed out or rebooted. Re-issue `SET_CONFIG`, then `RX_START` if continuous receive was previously active. Any TXs that were in-flight at the time are presumed not to have been transmitted.

### 14.3 MUST NOT

- Send commands with any reserved flag bit or reserved payload byte set.
- Assume any device-side state persists across disconnects.
- Assume response ordering across tags beyond what Section 11 guarantees.

---

## 15. Device Behavior Requirements

### 15.1 MUST

- Boot into UNCONFIGURED state.
- Reject unknown command types with `ERR(EUNKNOWN_CMD)` echoing the tag.
- Validate every `SET_CONFIG` parameter before writing to the radio. On validation failure the chip is not touched and an `ERR` is returned.
- Auto-enable LDRO on LoRa when `(2^SF) / BW_Hz > 16 ms`.
- Echo the command's tag on every `OK`, `ERR`, and `TX_DONE` response.
- Use tag `0x0000` for `RX` events and asynchronous `ERR` events.
- Reset the inactivity timer on every received frame, including frames that fail CRC or COBS decoding (the bytes arrived — the host is present).
- Emit exactly one of `OK` / `ERR` per command, then exactly one `TX_DONE` additionally for `TX` commands that received `OK`.
- Block the `OK` response to `SET_CONFIG` until the radio is fully configured and ready for subsequent commands (including any required calibration).

### 15.2 SHOULD

- Emit asynchronous `ERR(EFRAME)` when an inbound frame fails CRC or COBS decoding, to aid host-side diagnosis.
- Preserve the RX ring across TX operations (TX briefly interrupts RX, but queued RX events are not cleared).
- Use a display (if present) to reflect state: splash when UNCONFIGURED, connected status when CONFIGURED. Display behavior is not part of the protocol.

### 15.3 MUST NOT

- Persist any configuration, session state, tag state, or queue contents across reboots or disconnects.
- Initiate communication with the host (never send frames when the host has sent none).
- Enforce regulatory limits (TX power caps, duty cycle, band restrictions). The device is a transparent passthrough; regulatory compliance is the host's responsibility.
- Silently drop or modify packet payloads on TX or RX.

---

## 16. Version Compatibility

### 16.1 Protocol Version

`GET_INFO.proto_major` and `GET_INFO.proto_minor` identify the protocol version the device implements. This document specifies protocol version 1.0 (`proto_major = 1`, `proto_minor = 0`).

### 16.2 Major Version Changes

Different major versions are wire-incompatible. A host that does not recognize the device's reported `proto_major` MUST NOT attempt to use the device.

### 16.3 Minor Version Changes

Minor version increments are backward-compatible: they may add new message types, new capability bits, new modulations, new error codes, or new trailing fields on existing payloads. A host SHOULD continue to work with a device reporting a higher minor version, but will not have access to features added after its implementation.

### 16.4 Rules for Minor Version Extensions

Future minor versions MAY:
- Define new values in reserved ranges (message types, error codes, modulation IDs, chip IDs, capability bits).
- Append new fields to the end of existing payloads. Implementations use the frame length to determine the payload boundary; fields beyond what they understand are ignored.

Future minor versions MUST NOT:
- Change the meaning, encoding, or position of any existing field.
- Remove or renumber existing message types, error codes, chip IDs, modulation IDs, or capability bits.
- Change the semantics of existing commands or events.

---

## Appendix A — COBS Reference Implementation

```c
#include <stdint.h>
#include <stddef.h>

// Encode src[0..src_len-1] to dst. Returns bytes written to dst.
// dst must have room for src_len + ceil(src_len/254) + 1 bytes.
size_t cobs_encode(const uint8_t *src, size_t src_len, uint8_t *dst) {
    size_t r = 0, w = 1;
    size_t code_idx = 0;
    uint8_t code = 1;

    while (r < src_len) {
        if (src[r] == 0) {
            dst[code_idx] = code;
            code_idx = w++;
            code = 1;
        } else {
            dst[w++] = src[r];
            code++;
            if (code == 0xFF) {
                dst[code_idx] = code;
                code_idx = w++;
                code = 1;
            }
        }
        r++;
    }
    dst[code_idx] = code;
    return w;
}

// Decode src[0..src_len-1] to dst. Returns bytes written to dst,
// or 0 on decode error (corrupt input).
size_t cobs_decode(const uint8_t *src, size_t src_len, uint8_t *dst) {
    size_t r = 0, w = 0;
    while (r < src_len) {
        uint8_t code = src[r++];
        if (code == 0 || r + code - 1 > src_len) return 0;
        for (uint8_t i = 1; i < code; i++) dst[w++] = src[r++];
        if (code != 0xFF && r < src_len) dst[w++] = 0;
    }
    return w;
}
```

---

## Appendix B — CRC-16/CCITT-FALSE Reference Implementation

```c
#include <stdint.h>
#include <stddef.h>

uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}
```

Test vector: `crc16_ccitt_false("123456789", 9)` = `0x29B1`.

---

## Appendix C — Wire Examples

### C.1 Reading these examples

Every example below gives:

- **Direction and type**: `H→D` or `D→H`, followed by the message name and the tag in hex.
- **Decoded field values** (for messages with structured payloads): what each field means, not just raw bytes.
- **Pre-COBS body**: the `type | tag | payload` bytes, then a `| crc=0x....` marker, then the full pre-encoding frame `type | tag | payload | crc_lo crc_hi`.
- **Wire bytes**: the COBS-encoded frame followed by the trailing `0x00` delimiter. This is what actually moves over the transport.

All multi-byte integers are little-endian. All examples were generated by running the reference implementations in Appendices A and B against the specified input, and validated by round-trip.

Conventions used throughout:

- Tag values are chosen per section to keep examples distinct across the appendix. Real implementations use a monotonic counter from 1, skipping 0.
- Error cases assume the device is in the state described by the section intro (CONFIGURED unless noted otherwise).
- `...` in decoded fields means "same as previously shown, unchanged". Byte dumps are always complete.

---

### C.2 Basic sequences (happy path)

#### C.2.1 PING exchange

The simplest possible DongLoRa Protocol transaction. Host sends PING, device responds with OK.

```
H→D  PING  tag=0x0001
  pre-COBS: 01 01 00 | crc=0xc89d → 01 01 00 9D C8
  on wire:  03 01 01 03 9D C8 00

D→H  OK  tag=0x0001
  pre-COBS: 80 01 00 | crc=0xc4f7 → 80 01 00 F7 C4
  on wire:  03 80 01 03 F7 C4 00
```

#### C.2.2 GET_INFO exchange

Host queries device capabilities. Response describes an SX1262-based board with LoRa, FSK, and CAD; a 64-slot RX ring; a 16-deep TX queue; 150–960 MHz tuning; and −9 to +22 dBm TX power.

```
H→D  GET_INFO  tag=0x0002
  pre-COBS: 02 02 00 | crc=0xc49e → 02 02 00 9E C4
  on wire:  03 02 02 03 9E C4 00

D→H  OK  tag=0x0002
        proto_major: 1
        proto_minor: 0
        fw_major: 0
        fw_minor: 1
        fw_patch: 0
        radio_chip_id: 0x0002 (SX1262)
        capability_bitmap: 0x0000000000010003 (bits 0=LoRa, 1=FSK, 16=CAD)
        supported_sf_bitmap: 0x1FE0 (bits 5..12 = SF5..SF12)
        supported_bw_bitmap: 0x03FF (all sub-GHz BW enum values 0..9)
        max_payload_bytes: 255
        rx_queue_capacity: 64
        tx_queue_capacity: 16
        freq_min_hz: 150000000 (150 MHz)
        freq_max_hz: 960000000 (960 MHz)
        tx_power_min_dbm: -9
        tx_power_max_dbm: 22
        mcu_uid_len: 8
        mcu_uid: DE AD BE EF 01 23 45 67
        radio_uid_len: 0
        radio_uid: (empty)
  pre-COBS: 80 02 00 01 00 00 01 00 02 00 03 00 01 00 00 00 00 00 E0 1F FF 03 FF 00 40 00 10 00 80 D1 F0 08 00 70 38 39 F7 16 08 DE AD BE EF 01 23 45 67 00 | crc=0xa4fa → 80 02 00 01 00 00 01 00 02 00 03 00 01 00 00 00 00 00 E0 1F FF 03 FF 00 40 00 10 00 80 D1 F0 08 00 70 38 39 F7 16 08 DE AD BE EF 01 23 45 67 00 FA A4
  on wire:  03 80 02 02 01 01 02 01 02 02 02 03 02 01 01 01 01 01 06 E0 1F FF 03 FF 02 40 02 10 05 80 D1 F0 08 0F 70 38 39 F7 16 08 DE AD BE EF 01 23 45 67 03 FA A4 00
```

#### C.2.3 SET_CONFIG for LoRa (EU868, SF7, BW125, CR 4/5)

Standard LoRa configuration on EU868.1 MHz. Device applies and echoes the current params.

```
H→D  SET_CONFIG  tag=0x0003
        modulation_id: 0x01 (LoRa)
        freq_hz: 868100000 (868.1 MHz)
        sf: 7
        bw: 7 (125 kHz)
        cr: 0 (4/5)
        preamble_len: 8
        sync_word: 0x1424
        tx_power_dbm: 14
        header_mode: 0 (explicit)
        payload_crc: 1 (on)
        iq_invert: 0 (normal)
  pre-COBS: 03 03 00 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 | crc=0x1fd9 → 03 03 00 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 D9 1F
  on wire:  03 03 03 08 01 A0 27 BE 33 07 07 02 08 04 24 14 0E 02 01 03 D9 1F 00

D→H  OK  tag=0x0003
        result: 0 (APPLIED)
        owner: 1 (MINE)
        current_modulation: 0x01 (LoRa)
        current_params: [same 15 bytes echoed back]
  pre-COBS: 80 03 00 00 01 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 | crc=0x91c8 → 80 03 00 00 01 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 C8 91
  on wire:  03 80 03 01 09 01 01 A0 27 BE 33 07 07 02 08 04 24 14 0E 02 01 03 C8 91 00
```

#### C.2.4 TX with default CAD, packet "Hello"

Assumes C.2.3 has run. Host transmits `48 65 6C 6C 6F` ("Hello"). Device ACKs synchronously, performs CAD (~4 ms at SF7/BW125), transmits, and reports completion asynchronously. Airtime 30.976 ms = preamble (12.544 ms) + 18 payload symbols × 1.024 ms/symbol.

```
H→D  TX  tag=0x0004
        flags: 0x00 (CAD enabled)
        data: 48 65 6C 6C 6F ("Hello")
  pre-COBS: 04 04 00 00 48 65 6C 6C 6F | crc=0x4026 → 04 04 00 00 48 65 6C 6C 6F 26 40
  on wire:  03 04 04 01 08 48 65 6C 6C 6F 26 40 00

D→H  OK  tag=0x0004
  pre-COBS: 80 04 00 | crc=0x3b02 → 80 04 00 02 3B
  on wire:  03 80 04 03 02 3B 00

D→H  TX_DONE  tag=0x0004
        result: 0 (TRANSMITTED)
        airtime_us: 30976 (30.976 ms)
  pre-COBS: C1 04 00 00 00 79 00 00 | crc=0xfae3 → C1 04 00 00 00 79 00 00 E3 FA
  on wire:  03 C1 04 01 01 02 79 01 03 E3 FA 00
```

#### C.2.5 TX with skip_cad, packet "URGENT"

Host sets `skip_cad` to bypass CAD — typical for time-critical ACKs in protocols with their own MAC. No CAD time; straight to TX.

```
H→D  TX  tag=0x0005
        flags: 0x01 (skip_cad set)
        data: 55 52 47 45 4E 54 ("URGENT")
  pre-COBS: 04 05 00 01 55 52 47 45 4E 54 | crc=0x1cdb → 04 05 00 01 55 52 47 45 4E 54 DB 1C
  on wire:  03 04 05 0A 01 55 52 47 45 4E 54 DB 1C 00

D→H  OK  tag=0x0005
  pre-COBS: 80 05 00 | crc=0x0833 → 80 05 00 33 08
  on wire:  03 80 05 03 33 08 00

D→H  TX_DONE  tag=0x0005
        result: 0 (TRANSMITTED)
        airtime_us: 33792 (no CAD; straight to TX)
  pre-COBS: C1 05 00 00 00 84 00 00 | crc=0xe381 → C1 05 00 00 00 84 00 00 81 E3
  on wire:  03 C1 05 01 01 02 84 01 03 81 E3 00
```

#### C.2.6 Continuous RX loop

Host enters continuous RX. Device emits RX events as packets arrive. Host sends a periodic PING to keep the session alive, then eventually stops RX.

```
H→D  RX_START  tag=0x0006
  pre-COBS: 05 06 00 | crc=0x8dca → 05 06 00 CA 8D
  on wire:  03 05 06 03 CA 8D 00

D→H  OK  tag=0x0006
  pre-COBS: 80 06 00 | crc=0x5d60 → 80 06 00 60 5D
  on wire:  03 80 06 03 60 5D 00

D→H  RX  tag=0x0000
        rssi: -735 (-73.5 dBm)
        snr: 95 (+9.5 dB)
        freq_err: -125 (-125 Hz)
        timestamp_us: 42000000 (42.000 s since boot)
        crc_valid: 1 (pass)
        packets_dropped: 0
        origin: 0 (over-the-air)
        data: 01 02 03 04
  pre-COBS: C0 00 00 21 FD 5F 00 83 FF FF FF 80 DE 80 02 00 00 00 00 01 00 00 00 01 02 03 04 | crc=0x8eb9 → C0 00 00 21 FD 5F 00 83 FF FF FF 80 DE 80 02 00 00 00 00 01 00 00 00 01 02 03 04 B9 8E
  on wire:  02 C0 01 04 21 FD 5F 09 83 FF FF FF 80 DE 80 02 01 01 01 02 01 01 01 07 01 02 03 04 B9 8E 00

H→D  PING  tag=0x0007
  pre-COBS: 01 07 00 | crc=0x623b → 01 07 00 3B 62
  on wire:  03 01 07 03 3B 62 00

D→H  OK  tag=0x0007
  pre-COBS: 80 07 00 | crc=0x6e51 → 80 07 00 51 6E
  on wire:  03 80 07 03 51 6E 00

H→D  RX_STOP  tag=0x0008
  pre-COBS: 06 08 00 | crc=0xf795 → 06 08 00 95 F7
  on wire:  03 06 08 03 95 F7 00

D→H  OK  tag=0x0008
  pre-COBS: 80 08 00 | crc=0x7e6f → 80 08 00 6F 7E
  on wire:  03 80 08 03 6F 7E 00
```

---

### C.3 Pipelining and batching

#### C.3.1 Three pipelined TXs

Host issues three TXs without waiting for intermediate responses. Each gets `OK` synchronously as it is enqueued; `TX_DONE` events arrive later in submission order.

```
H→D  TX  tag=0x0064   (data: 00 41 = flags + "A")
  pre-COBS: 04 64 00 00 41 | crc=0x53cc → 04 64 00 00 41 CC 53
  on wire:  03 04 64 01 04 41 CC 53 00

H→D  TX  tag=0x0065   (data: 00 42 = flags + "B")
  pre-COBS: 04 65 00 00 42 | crc=0x151b → 04 65 00 00 42 1B 15
  on wire:  03 04 65 01 04 42 1B 15 00

H→D  TX  tag=0x0066   (data: 00 43 = flags + "C")
  pre-COBS: 04 66 00 00 43 | crc=0x9ee6 → 04 66 00 00 43 E6 9E
  on wire:  03 04 66 01 04 43 E6 9E 00

D→H  OK  tag=0x0064
  pre-COBS: 80 64 00 | crc=0x3028 → 80 64 00 28 30
  on wire:  03 80 64 03 28 30 00

D→H  OK  tag=0x0065
  pre-COBS: 80 65 00 | crc=0x0319 → 80 65 00 19 03
  on wire:  03 80 65 03 19 03 00

D→H  OK  tag=0x0066
  pre-COBS: 80 66 00 | crc=0x564a → 80 66 00 4A 56
  on wire:  03 80 66 03 4A 56 00

D→H  TX_DONE  tag=0x0064   (result=0, airtime=25000 us)
  pre-COBS: C1 64 00 00 A8 61 00 00 | crc=0xcc8e → C1 64 00 00 A8 61 00 00 8E CC
  on wire:  03 C1 64 01 03 A8 61 01 03 8E CC 00

D→H  TX_DONE  tag=0x0065
  pre-COBS: C1 65 00 00 A8 61 00 00 | crc=0x74ef → C1 65 00 00 A8 61 00 00 EF 74
  on wire:  03 C1 65 01 03 A8 61 01 03 EF 74 00

D→H  TX_DONE  tag=0x0066
  pre-COBS: C1 66 00 00 A8 61 00 00 | crc=0xac6d → C1 66 00 00 A8 61 00 00 6D AC
  on wire:  03 C1 66 01 03 A8 61 01 03 6D AC 00
```

#### C.3.2 Batched frames in a single transport write (narrative)

A host writing three small frames in quick succession may see them concatenated into a single USB bulk transfer, one BLE notification, or one TCP packet. The byte stream is:

```
[frame1 wire bytes] [frame2 wire bytes] [frame3 wire bytes]
```

For the C.3.1 example, the first three `TX` frames batched look like:

```
03 04 64 01 04 41 CC 53 00  03 04 65 01 04 42 1B 15 00  03 04 66 01 04 43 E6 9E 00
```

That is, 27 bytes of raw transport data containing three DongLoRa Protocol frames. The receiver's COBS delimiter scanner handles this naturally: it reads bytes into a frame buffer until `0x00`, decodes that frame, flushes the buffer, and continues. No additional framing or length-prefix is involved.

---

### C.4 State transitions

#### C.4.1 Reconfigure while RX is active

Host is in continuous RX, receives a packet, then reconfigures (changing SF from 7 to 9). Device aborts RX, clears the ring, reconfigures, and returns to standby. Host must re-issue `RX_START` to resume.

```
H→D  RX_START  tag=0x000A
  pre-COBS: 05 0A 00 | crc=0xc8a7 → 05 0A 00 A7 C8
  on wire:  03 05 0A 03 A7 C8 00

D→H  OK  tag=0x000A
  pre-COBS: 80 0A 00 | crc=0x180d → 80 0A 00 0D 18
  on wire:  03 80 0A 03 0D 18 00

D→H  RX  tag=0x0000
        rssi: -820, snr: 80, freq_err: 0, timestamp_us: 50000000,
        crc_valid: 1, packets_dropped: 0, origin: 0, data: AA BB
  pre-COBS: C0 00 00 CC FC 50 00 00 00 00 00 80 F0 FA 02 00 00 00 00 01 00 00 00 AA BB | crc=0xc054 → C0 00 00 CC FC 50 00 00 00 00 00 80 F0 FA 02 00 00 00 00 01 00 00 00 AA BB 54 C0
  on wire:  02 C0 01 04 CC FC 50 01 01 01 01 05 80 F0 FA 02 01 01 01 02 01 01 01 05 AA BB 54 C0 00

H→D  SET_CONFIG  tag=0x000B   (LoRa, SF=9, other params same as C.2.3)
  pre-COBS: 03 0B 00 01 A0 27 BE 33 09 07 00 08 00 24 14 0E 00 01 00 | crc=0xbdcc → 03 0B 00 01 A0 27 BE 33 09 07 00 08 00 24 14 0E 00 01 00 CC BD
  on wire:  03 03 0B 08 01 A0 27 BE 33 09 07 02 08 04 24 14 0E 02 01 03 CC BD 00

D→H  OK  tag=0x000B
        result: 0 (APPLIED), owner: 1 (MINE), current: LoRa SF=9 ...
  pre-COBS: 80 0B 00 00 01 01 A0 27 BE 33 09 07 00 08 00 24 14 0E 00 01 00 | crc=0x7f0b → 80 0B 00 00 01 01 A0 27 BE 33 09 07 00 08 00 24 14 0E 00 01 00 0B 7F
  on wire:  03 80 0B 01 09 01 01 A0 27 BE 33 09 07 02 08 04 24 14 0E 02 01 03 0B 7F 00

(radio is now idle; host must re-start RX)

H→D  RX_START  tag=0x000C
  pre-COBS: 05 0C 00 | crc=0x6201 → 05 0C 00 01 62
  on wire:  03 05 0C 03 01 62 00

D→H  OK  tag=0x000C
  pre-COBS: 80 0C 00 | crc=0xb2ab → 80 0C 00 AB B2
  on wire:  03 80 0C 03 AB B2 00
```

#### C.4.2 Reconfigure with pending TXs — they are cancelled

Host queues two TXs, then before they complete, sends `SET_CONFIG`. Both queued TXs emit `TX_DONE(CANCELLED)`. The `SET_CONFIG` then applies.

```
H→D  TX  tag=0x0014   (data = "first")
  pre-COBS: 04 14 00 00 66 69 72 73 74 | crc=0x1dc9 → 04 14 00 00 66 69 72 73 74 C9 1D
  on wire:  03 04 14 01 08 66 69 72 73 74 C9 1D 00

D→H  OK  tag=0x0014
  pre-COBS: 80 14 00 | crc=0x3871 → 80 14 00 71 38
  on wire:  03 80 14 03 71 38 00

H→D  TX  tag=0x0015   (data = "second")
  pre-COBS: 04 15 00 00 73 65 63 6F 6E 64 | crc=0xa89a → 04 15 00 00 73 65 63 6F 6E 64 9A A8
  on wire:  03 04 15 01 09 73 65 63 6F 6E 64 9A A8 00

D→H  OK  tag=0x0015
  pre-COBS: 80 15 00 | crc=0x0b40 → 80 15 00 40 0B
  on wire:  03 80 15 03 40 0B 00

H→D  SET_CONFIG  tag=0x0016   (LoRa SF=10 — different from current config)
  pre-COBS: 03 16 00 01 A0 27 BE 33 0A 07 00 08 00 24 14 0E 00 01 00 | crc=0xc630 → 03 16 00 01 A0 27 BE 33 0A 07 00 08 00 24 14 0E 00 01 00 30 C6
  on wire:  03 03 16 08 01 A0 27 BE 33 0A 07 02 08 04 24 14 0E 02 01 03 30 C6 00

D→H  TX_DONE  tag=0x0014
        result: 2 (CANCELLED), airtime_us: 0
  pre-COBS: C1 14 00 02 00 00 00 00 | crc=0xcf82 → C1 14 00 02 00 00 00 00 82 CF
  on wire:  03 C1 14 02 02 01 01 01 03 82 CF 00

D→H  TX_DONE  tag=0x0015
        result: 2 (CANCELLED), airtime_us: 0
  pre-COBS: C1 15 00 02 00 00 00 00 | crc=0x77e3 → C1 15 00 02 00 00 00 00 E3 77
  on wire:  03 C1 15 02 02 01 01 01 03 E3 77 00

D→H  OK  tag=0x0016
        result: 0 (APPLIED), owner: 1 (MINE), current: LoRa SF=10 ...
  pre-COBS: 80 16 00 00 01 01 A0 27 BE 33 0A 07 00 08 00 24 14 0E 00 01 00 | crc=0x3264 → 80 16 00 00 01 01 A0 27 BE 33 0A 07 00 08 00 24 14 0E 00 01 00 64 32
  on wire:  03 80 16 01 09 01 01 A0 27 BE 33 0A 07 02 08 04 24 14 0E 02 01 03 64 32 00
```

#### C.4.3 Inactivity timeout and recovery

Host goes silent for more than 1000 ms. Device transitions to UNCONFIGURED. When host returns and tries to TX, it gets `ERR(ENOTCONFIGURED)`. Host re-issues `SET_CONFIG` to recover.

```
H→D  TX  tag=0x001E   (data = "before")
  pre-COBS: 04 1E 00 00 62 65 66 6F 72 65 | crc=0x197f → 04 1E 00 00 62 65 66 6F 72 65 7F 19
  on wire:  03 04 1E 01 09 62 65 66 6F 72 65 7F 19 00

D→H  OK  tag=0x001E
  pre-COBS: 80 1E 00 | crc=0xd7ba → 80 1E 00 BA D7
  on wire:  03 80 1E 03 BA D7 00

D→H  TX_DONE  tag=0x001E   (TRANSMITTED, airtime 30976 us)
  pre-COBS: C1 1E 00 00 00 79 00 00 | crc=0x3ed6 → C1 1E 00 00 00 79 00 00 D6 3E
  on wire:  03 C1 1E 01 01 02 79 01 03 D6 3E 00

        [ host sleeps 1.5 s — no frames sent ]
        [ device's inactivity timer expires → UNCONFIGURED ]

H→D  TX  tag=0x001F   (data = "after")
  pre-COBS: 04 1F 00 00 61 66 74 65 72 | crc=0x03ef → 04 1F 00 00 61 66 74 65 72 EF 03
  on wire:  03 04 1F 01 08 61 66 74 65 72 EF 03 00

D→H  ERR  tag=0x001F
        code: 0x0003 (ENOTCONFIGURED)
  pre-COBS: 81 1F 00 03 00 | crc=0x0397 → 81 1F 00 03 00 97 03
  on wire:  03 81 1F 02 03 03 97 03 00

        (host realises session is lost; re-configures)

H→D  SET_CONFIG  tag=0x0020   (LoRa SF=7, as before)
  pre-COBS: 03 20 00 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 | crc=0xea74 → 03 20 00 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 74 EA
  on wire:  03 03 20 08 01 A0 27 BE 33 07 07 02 08 04 24 14 0E 02 01 03 74 EA 00

D→H  OK  tag=0x0020   (APPLIED, MINE)
  pre-COBS: 80 20 00 00 01 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 | crc=0x19bb → 80 20 00 00 01 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 BB 19
  on wire:  03 80 20 01 09 01 01 A0 27 BE 33 07 07 02 08 04 24 14 0E 02 01 03 BB 19 00

H→D  TX  tag=0x0021   (retry)
  pre-COBS: 04 21 00 00 61 66 74 65 72 | crc=0xdb22 → 04 21 00 00 61 66 74 65 72 22 DB
  on wire:  03 04 21 01 08 61 66 74 65 72 22 DB 00

D→H  OK  tag=0x0021
  pre-COBS: 80 21 00 | crc=0xc211 → 80 21 00 11 C2
  on wire:  03 80 21 03 11 C2 00

D→H  TX_DONE  tag=0x0021   (TRANSMITTED, airtime 30976 us)
  pre-COBS: C1 21 00 00 00 79 00 00 | crc=0xedb2 → C1 21 00 00 00 79 00 00 B2 ED
  on wire:  03 C1 21 01 01 02 79 01 03 B2 ED 00
```

---

### C.5 Error responses

#### C.5.1 TX attempted in UNCONFIGURED state

Host sends TX without first calling `SET_CONFIG`. Device returns `ERR(ENOTCONFIGURED)`.

```
H→D  TX  tag=0x0028   (data = "hi")
  pre-COBS: 04 28 00 00 68 69 | crc=0x7d24 → 04 28 00 00 68 69 24 7D
  on wire:  03 04 28 01 05 68 69 24 7D 00

D→H  ERR  tag=0x0028
        code: 0x0003 (ENOTCONFIGURED)
  pre-COBS: 81 28 00 03 00 | crc=0x7e53 → 81 28 00 03 00 53 7E
  on wire:  03 81 28 02 03 03 53 7E 00
```

#### C.5.2 TX with empty payload data

Host sends TX with only the flags byte and no data. Device returns `ERR(ELENGTH)` because data must be at least 1 byte.

```
H→D  TX  tag=0x0029
        flags: 0
        data: (empty — invalid)
  pre-COBS: 04 29 00 00 | crc=0x5666 → 04 29 00 00 66 56
  on wire:  03 04 29 01 03 66 56 00

D→H  ERR  tag=0x0029
        code: 0x0002 (ELENGTH)
  pre-COBS: 81 29 00 02 00 | crc=0x3bd6 → 81 29 00 02 00 D6 3B
  on wire:  03 81 29 02 02 03 D6 3B 00
```

#### C.5.3 TX with reserved flag bit set

Host sets bit 1 of `flags` (reserved). Device returns `ERR(EPARAM)`.

```
H→D  TX  tag=0x002A
        flags: 0x02 (bit 1 is reserved)
        data: 68 69
  pre-COBS: 04 2A 00 02 68 69 | crc=0x57c7 → 04 2A 00 02 68 69 C7 57
  on wire:  03 04 2A 06 02 68 69 C7 57 00

D→H  ERR  tag=0x002A
        code: 0x0001 (EPARAM)
  pre-COBS: 81 2A 00 01 00 | crc=0xf559 → 81 2A 00 01 00 59 F5
  on wire:  03 81 2A 02 01 03 59 F5 00
```

#### C.5.4 TX queue full (EBUSY)

Host has pipelined `tx_queue_capacity` TXs and none have completed. The next TX returns `ERR(EBUSY)`. Host waits for a `TX_DONE` to free a slot, then retries with a **new** tag.

```
H→D  TX  tag=0x002B   (queue already full)
  pre-COBS: 04 2B 00 00 6F 76 65 72 66 6C 6F 77 | crc=0x4824 → 04 2B 00 00 6F 76 65 72 66 6C 6F 77 24 48
  on wire:  03 04 2B 01 0B 6F 76 65 72 66 6C 6F 77 24 48 00

D→H  ERR  tag=0x002B
        code: 0x0006 (EBUSY)
  pre-COBS: 81 2B 00 06 00 | crc=0x1a7a → 81 2B 00 06 00 7A 1A
  on wire:  03 81 2B 02 06 03 7A 1A 00
```

#### C.5.5 TX collision (CHANNEL_BUSY) and retry

CAD detects another transmitter. TX is aborted before airtime. Host backs off (randomised 20–100 ms) and retries with a new tag.

```
H→D  TX  tag=0x0032   (data = "retry-me")
  pre-COBS: 04 32 00 00 72 65 74 72 79 2D 6D 65 | crc=0x0d16 → 04 32 00 00 72 65 74 72 79 2D 6D 65 16 0D
  on wire:  03 04 32 01 0B 72 65 74 72 79 2D 6D 65 16 0D 00

D→H  OK  tag=0x0032
  pre-COBS: 80 32 00 | crc=0x9431 → 80 32 00 31 94
  on wire:  03 80 32 03 31 94 00

D→H  TX_DONE  tag=0x0032
        result: 1 (CHANNEL_BUSY), airtime_us: 0
  pre-COBS: C1 32 00 01 00 00 00 00 | crc=0xee83 → C1 32 00 01 00 00 00 00 83 EE
  on wire:  03 C1 32 02 01 01 01 01 03 83 EE 00

        [ random backoff ]

H→D  TX  tag=0x0033   (same data, NEW tag)
  pre-COBS: 04 33 00 00 72 65 74 72 79 2D 6D 65 | crc=0xd55f → 04 33 00 00 72 65 74 72 79 2D 6D 65 5F D5
  on wire:  03 04 33 01 0B 72 65 74 72 79 2D 6D 65 5F D5 00

D→H  OK  tag=0x0033
  pre-COBS: 80 33 00 | crc=0xa700 → 80 33 00 00 A7
  on wire:  03 80 33 01 02 A7 00

D→H  TX_DONE  tag=0x0033
        result: 0 (TRANSMITTED), airtime_us: 30976
  pre-COBS: C1 33 00 00 00 79 00 00 | crc=0xba2a → C1 33 00 00 00 79 00 00 2A BA
  on wire:  03 C1 33 01 01 02 79 01 03 2A BA 00
```

Note: the `OK` response for tag=0x0033 shows a COBS encoding quirk: the CRC is `0xA700`, so the low byte is `0x00`. COBS encodes this by inserting a `0x02` code byte that splits the run around the zero.

#### C.5.6 Unknown command type byte

Host sends a frame with `type = 0x10` (in the reserved H→D range). Device returns `ERR(EUNKNOWN_CMD)` with the received tag echoed.

```
H→D  type=0x10  tag=0x003C
        type: 0x10 (reserved, not yet assigned)
        payload: DE AD
  pre-COBS: 10 3C 00 DE AD | crc=0x24e2 → 10 3C 00 DE AD E2 24
  on wire:  03 10 3C 05 DE AD E2 24 00

D→H  ERR  tag=0x003C
        code: 0x0005 (EUNKNOWN_CMD)
  pre-COBS: 81 3C 00 05 00 | crc=0x05a3 → 81 3C 00 05 00 A3 05
  on wire:  03 81 3C 02 05 03 A3 05 00
```

#### C.5.7 SET_CONFIG with truncated LoRa params

Host declares LoRa modulation but includes only 10 bytes of params instead of the required 15. Device returns `ERR(ELENGTH)`.

```
H→D  SET_CONFIG  tag=0x0046
        modulation_id: 0x01 (LoRa)
        params: (10 bytes — should be 15)
  pre-COBS: 03 46 00 01 00 00 00 00 00 00 00 00 00 00 | crc=0x293b → 03 46 00 01 00 00 00 00 00 00 00 00 00 00 3B 29
  on wire:  03 03 46 02 01 01 01 01 01 01 01 01 01 01 03 3B 29 00

D→H  ERR  tag=0x0046
        code: 0x0002 (ELENGTH)
  pre-COBS: 81 46 00 02 00 | crc=0xb6ea → 81 46 00 02 00 EA B6
  on wire:  03 81 46 02 02 03 EA B6 00
```

#### C.5.8 SET_CONFIG with frequency outside chip range

Host requests 2.45 GHz on a sub-GHz-only SX1262 (advertised range 150–960 MHz). Device returns `ERR(EPARAM)`.

```
H→D  SET_CONFIG  tag=0x0047
        modulation_id: 0x01 (LoRa)
        freq_hz: 2450000000 (2.45 GHz — outside 150 MHz–960 MHz)
        (other params as normal)
  pre-COBS: 03 47 00 01 80 08 08 92 07 07 00 08 00 24 14 0E 00 01 00 | crc=0x4934 → 03 47 00 01 80 08 08 92 07 07 00 08 00 24 14 0E 00 01 00 34 49
  on wire:  03 03 47 08 01 80 08 08 92 07 07 02 08 04 24 14 0E 02 01 03 34 49 00

D→H  ERR  tag=0x0047
        code: 0x0001 (EPARAM)
  pre-COBS: 81 47 00 01 00 | crc=0x950d → 81 47 00 01 00 0D 95
  on wire:  03 81 47 02 01 03 0D 95 00
```

#### C.5.9 SET_CONFIG with unsupported modulation

Host requests FLRC (2.4 GHz-only modulation) on a sub-GHz SX1262. Capability bitmap does not advertise FLRC. Device returns `ERR(EMODULATION)`.

```
H→D  SET_CONFIG  tag=0x0048
        modulation_id: 0x04 (FLRC — not supported on SX1262)
        params: [13 bytes, ignored because modulation is unsupported]
  pre-COBS: 03 48 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00 | crc=0x96c2 → 03 48 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00 C2 96
  on wire:  03 03 48 02 04 01 01 01 01 01 01 01 01 01 01 01 01 03 C2 96 00

D→H  ERR  tag=0x0048
        code: 0x0004 (EMODULATION)
  pre-COBS: 81 48 00 04 00 | crc=0xbe16 → 81 48 00 04 00 16 BE
  on wire:  03 81 48 02 04 03 16 BE 00
```

#### C.5.10 Radio hardware fault during SET_CONFIG

Radio SPI fails to respond during the configuration sequence. Device returns `ERR(ERADIO)` synchronously, transitions to UNCONFIGURED, and places the radio in its safest reachable state.

```
H→D  SET_CONFIG  tag=0x0049   (LoRa, standard params)
  pre-COBS: 03 49 00 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 | crc=0xe56a → 03 49 00 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 6A E5
  on wire:  03 03 49 08 01 A0 27 BE 33 07 07 02 08 04 24 14 0E 02 01 03 6A E5 00

D→H  ERR  tag=0x0049
        code: 0x0101 (ERADIO)
  pre-COBS: 81 49 00 01 01 | crc=0x2776 → 81 49 00 01 01 76 27
  on wire:  03 81 49 05 01 01 76 27 00
```

---

### C.6 Asynchronous events

Async events carry `tag = 0x0000`. They may appear interleaved anywhere in the D→H stream.

#### C.6.1 RX with good CRC

See C.2.6 above for an example within the continuous-RX flow.

#### C.6.2 RX with crc_valid=0 (corrupt packet delivered anyway)

Radio reports a packet, but PHY CRC failed. Device delivers the bytes with `crc_valid = 0`; the host decides whether to discard. RSSI and SNR reported but may not be meaningful.

```
D→H  RX  tag=0x0000
        rssi: -1050 (-105.0 dBm — weak signal)
        snr: -80 (-8.0 dB — negative SNR)
        freq_err: 2200 (2.2 kHz offset)
        timestamp_us: 120000000 (120.000 s)
        crc_valid: 0 (FAIL — bytes may be corrupt)
        packets_dropped: 0
        origin: 0 (over-the-air)
        data: 41 42 FF 00 CC
  pre-COBS: C0 00 00 E6 FB B0 FF 98 08 00 00 00 0E 27 07 00 00 00 00 00 00 00 00 41 42 FF 00 CC | crc=0x3c04 → C0 00 00 E6 FB B0 FF 98 08 00 00 00 0E 27 07 00 00 00 00 00 00 00 00 41 42 FF 00 CC 04 3C
  on wire:  02 C0 01 07 E6 FB B0 FF 98 08 01 01 04 0E 27 07 01 01 01 01 01 01 01 04 41 42 FF 04 CC 04 3C 00
```

#### C.6.3 RX following buffer overrun

Host was slow draining. Three packets arrived and were dropped because the ring was full. The next successfully-delivered RX reports `packets_dropped = 3`. The counter resets to 0 after this delivery.

```
D→H  RX  tag=0x0000
        rssi: -730 (-73.0 dBm)
        snr: 100 (+10.0 dB)
        freq_err: 50 (50 Hz)
        timestamp_us: 130500000
        crc_valid: 1 (pass)
        packets_dropped: 3 (three lost since previous delivery)
        origin: 0
        data: 01 02
  pre-COBS: C0 00 00 26 FD 64 00 32 00 00 00 A0 45 C7 07 00 00 00 00 01 03 00 00 01 02 | crc=0x86a7 → C0 00 00 26 FD 64 00 32 00 00 00 A0 45 C7 07 00 00 00 00 01 03 00 00 01 02 A7 86
  on wire:  02 C0 01 04 26 FD 64 02 32 01 01 05 A0 45 C7 07 01 01 01 03 01 03 01 05 01 02 A7 86 00
```

#### C.6.4 Malformed incoming frame triggers async ERR(EFRAME)

Host (or noise on the line) sends a frame whose CRC does not match the body. Device discards the frame and emits an async `ERR(EFRAME)` with `tag = 0`. The liveness timer is still reset — the bytes arrived, so the host is present.

```
D→H  ERR  tag=0x0000
        code: 0x0102 (EFRAME)
  pre-COBS: 81 00 00 02 01 | crc=0xefce → 81 00 00 02 01 CE EF
  on wire:  02 81 01 05 02 01 CE EF 00
```

Note: the same frame type (`ERR`, `0x81`) is used here as in synchronous error responses; the tag (`0x0000`) is the only discriminator. Host code distinguishes async faults from command responses by the tag value.

#### C.6.5 Asynchronous radio fault

Radio chip asserts an unexpected IRQ (e.g., an RxTimeout from a stale `RX_START` that the firmware already reset). No originating command. Device emits `ERR(ERADIO)` with `tag = 0`.

```
D→H  ERR  tag=0x0000
        code: 0x0101 (ERADIO)
  pre-COBS: 81 00 00 01 01 | crc=0xba9d → 81 00 00 01 01 9D BA
  on wire:  02 81 01 05 01 01 9D BA 00
```

---

### C.7 Multi-client scenarios (informative)

These examples assume firmware with multi-client capability bit 32 set. Single-client deployments will never see these responses.

#### C.7.1 Second client finds matching config (ALREADY_MATCHED)

Client A has locked LoRa SF7/BW125. Client B connects and requests identical params. Device reports `ALREADY_MATCHED` with `owner = OTHER`; Client B may proceed without changing anything.

```
H→D  SET_CONFIG  tag=0x0050   (from Client B; identical to Client A's config)
  pre-COBS: 03 50 00 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 | crc=0x16cb → 03 50 00 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 CB 16
  on wire:  03 03 50 08 01 A0 27 BE 33 07 07 02 08 04 24 14 0E 02 01 03 CB 16 00

D→H  OK  tag=0x0050
        result: 1 (ALREADY_MATCHED)
        owner: 2 (OTHER — Client A holds the lock)
        current_modulation: 0x01 (LoRa)
        current_params: [SF=7, BW=125 kHz, ... same as request]
  pre-COBS: 80 50 00 01 02 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 | crc=0xd834 → 80 50 00 01 02 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 34 D8
  on wire:  03 80 50 0A 01 02 01 A0 27 BE 33 07 07 02 08 04 24 14 0E 02 01 03 34 D8 00
```

#### C.7.2 Second client finds mismatched config (LOCKED_MISMATCH)

Client A is locked on SF7. Client B requests SF9 with different TX power. Device rejects the change; returns the actual active params so Client B can diff and decide what to do.

```
H→D  SET_CONFIG  tag=0x0051   (from Client B; sf=9, tx_power_dbm=17)
  pre-COBS: 03 51 00 01 A0 27 BE 33 09 07 00 08 00 24 14 11 00 01 00 | crc=0x7ac9 → 03 51 00 01 A0 27 BE 33 09 07 00 08 00 24 14 11 00 01 00 C9 7A
  on wire:  03 03 51 08 01 A0 27 BE 33 09 07 02 08 04 24 14 11 02 01 03 C9 7A 00

D→H  OK  tag=0x0051
        result: 2 (LOCKED_MISMATCH)
        owner: 2 (OTHER)
        current_modulation: 0x01 (LoRa)
        current_params: [SF=7, tx_power=14 — Client A's setting, unchanged]
  pre-COBS: 80 51 00 02 02 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 | crc=0xedf5 → 80 51 00 02 02 01 A0 27 BE 33 07 07 00 08 00 24 14 0E 00 01 00 F5 ED
  on wire:  03 80 51 0A 02 02 01 A0 27 BE 33 07 07 02 08 04 24 14 0E 02 01 03 F5 ED 00
```

#### C.7.3 Cross-client TX loopback (origin=1)

Client A transmits. Client B, which is also RX-active on the same device, sees the packet as an RX event with the `origin = 1` flag set, distinguishing it from over-the-air traffic.

```
H→D  TX  tag=0x005A   (from Client A; data = "BROADCAST")
  pre-COBS: 04 5A 00 00 42 52 4F 41 44 43 41 53 54 | crc=0x7914 → 04 5A 00 00 42 52 4F 41 44 43 41 53 54 14 79
  on wire:  03 04 5A 01 0C 42 52 4F 41 44 43 41 53 54 14 79 00

D→H  OK  tag=0x005A   (to Client A only)
  pre-COBS: 80 5A 00 | crc=0x16b2 → 80 5A 00 B2 16
  on wire:  03 80 5A 03 B2 16 00

D→H  TX_DONE  tag=0x005A   (to Client A only; TRANSMITTED)
  pre-COBS: C1 5A 00 00 00 D8 00 00 | crc=0xa850 → C1 5A 00 00 00 D8 00 00 50 A8
  on wire:  03 C1 5A 01 01 02 D8 01 03 50 A8 00

D→H  RX  tag=0x0000   (to Client B only; local loopback, not OTA)
        rssi: 0 (n/a for local loopback)
        snr: 0
        freq_err: 0
        timestamp_us: 180000000
        crc_valid: 1 (local TX, assumed valid)
        packets_dropped: 0
        origin: 1 (local loopback — NOT over the air)
        data: 42 52 4F 41 44 43 41 53 54 ("BROADCAST")
  pre-COBS: C0 00 00 00 00 00 00 00 00 00 00 00 95 BA 0A 00 00 00 00 01 00 00 01 42 52 4F 41 44 43 41 53 54 | crc=0x00e0 → C0 00 00 00 00 00 00 00 00 00 00 00 95 BA 0A 00 00 00 00 01 00 00 01 42 52 4F 41 44 43 41 53 54 E0 00
  on wire:  02 C0 01 01 01 01 01 01 01 01 01 01 04 95 BA 0A 01 01 01 02 01 01 0C 01 42 52 4F 41 44 43 41 53 54 E0 01 00
```

---

### C.8 COBS edge cases

#### C.8.1 Frame body contains no 0x00 bytes

When the unencoded body has no zero bytes, COBS simply prepends one code byte giving the run length + 1. No internal code bytes are inserted.

```
H→D  PING  tag=0x0101
        (tag bytes 0x01 0x01 — no zeros anywhere in body)
  pre-COBS: 01 01 01 | crc=0xd8bc → 01 01 01 BC D8
  on wire:  06 01 01 01 BC D8 00
```

The leading `06` means "next zero (or end of frame) is 6 bytes away".

#### C.8.2 Frame body contains many 0x00 bytes

RX event with all-zero metadata and a 3-byte all-zero payload. COBS must replace each zero with a run-length code byte. This is the opposite extreme from C.8.1: many short runs, many code bytes.

```
D→H  RX  tag=0x0000
        rssi: 0, snr: 0, freq_err: 0
        timestamp_us: 256 (0x100)
        crc_valid: 1, packets_dropped: 0, origin: 0
        data: 00 00 00
  pre-COBS: C0 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 01 00 00 00 00 00 00 | crc=0x74b4 → C0 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 01 00 00 00 00 00 00 B4 74
  on wire:  02 C0 01 01 01 01 01 01 01 01 01 01 02 01 01 01 01 01 01 02 01 01 01 01 01 01 03 B4 74 00
```

The frequent `01` bytes in the wire output are COBS code bytes pointing "next zero is 1 byte away" — each represents a single zero in the original body.

#### C.8.3 254-byte non-zero run triggers a 0xFF code byte

When the body contains a run of 254 or more non-zero bytes, COBS inserts an extra code byte. This is the only case where encoding adds more than a fixed constant — it adds `ceil(N / 254)` overhead bytes for an N-byte run.

Example: a TX command with tag=0x0101, skip_cad flag, and 253 bytes of non-zero data. Total pre-COBS body is 259 bytes with no zeros (header, flags byte, all 253 data bytes, and both CRC bytes happen to be non-zero).

```
  Body composition (259 bytes, all non-zero):
    type:  04                           (1 byte)
    tag:   01 01                        (2 bytes)
    flags: 01                           (1 byte)
    data:  01 02 03 ... FD              (253 bytes, values 0x01..0xFD)
    crc:   53 46                        (2 bytes; CRC = 0x4653)

  COBS output (261 bytes):
    Position 0:   FF                    ← code byte: "next zero (or end of frame)
                                          would be 255 bytes away; bail and start
                                          a fresh run here"
    Positions 1..254:  04 01 01 01 01 02 03 ... FC   (first 254 source bytes)
    Position 255:      06                             ← new code byte
    Positions 256..260: FD 53 46                      (remaining 5 source bytes:
                                                        the last data byte FD,
                                                        plus CRC 53 46)
                                          (code says "6 bytes" = 5 data + the
                                           virtual end-of-frame position)

  Wire: 261 encoded bytes + 1 trailing 0x00 delimiter = 262 bytes total.
```

**Buffer sizing consequence**: a host encoding an N-byte frame needs at most `N + ceil(N / 254) + 1` bytes of output buffer (plus the trailing delimiter). For any frame under 254 bytes, overhead is exactly 1 byte (the leading code byte) plus the trailing delimiter. At exactly 254 bytes or more, a second code byte is added. At 508+, a third. Real DongLoRa Protocol frames almost always fall in the single-overhead-byte range.

---

### C.9 Parser resynchronization (narrative)

#### C.9.1 Mid-stream connection

A host process opens the transport while the device is mid-frame (for example, a previous host process was killed and a new one is starting up). The incoming byte stream looks like:

```
... 14 0E 02 01 03 0B 7F 00 03 05 0C 03 01 62 00 ...
       |← tail of previous frame →|  |← next complete frame →|
```

The new host discards incoming bytes until it sees a `0x00`. That `0x00` closes whatever partial frame was in flight when the host started reading (which is discarded). Reading resumes with the byte immediately following: `03 05 0C 03 01 62 00` is the next complete frame, which COBS-decodes cleanly and verifies CRC. Normal operation proceeds.

No special handshake, no timeout — the self-synchronising property of COBS makes mid-stream recovery deterministic and instantaneous.

#### C.9.2 Single-frame corruption

The host receives three frames in rapid succession. The middle one has a bit error in transit (say, flipped by a noisy USB cable) that corrupts one of the data bytes. The host:

1. Reads to the first `0x00`, decodes the first frame, verifies CRC → OK, processes it.
2. Reads to the second `0x00`, decodes the second frame, **CRC check fails** → discards the frame silently. Does not emit any response (it cannot determine the originating tag reliably from a corrupted frame).
3. Reads to the third `0x00`, decodes the third frame, verifies CRC → OK, processes it.

The outstanding tag of the discarded frame remains open on the host side. The host's per-command timeout (Section 14.2) will eventually fire, at which point the host may re-issue the command with a new tag. In practice, transport-level bit errors on USB are very rare; this path is defensive rather than routine.

The device, in the symmetrical case, emits an async `ERR(EFRAME)` with tag `0x0000` (§C.6.4) as a diagnostic hint but does not directly signal which command was lost — the corrupted bytes are, by definition, not reliably parseable.

## Appendix D — Glossary

**Airtime**: The duration a packet occupies the radio channel during transmission.

**CAD**: Channel Activity Detection. A LoRa radio primitive that briefly listens for a LoRa preamble and reports busy/clear.

**COBS**: Consistent Overhead Byte Stuffing. A framing technique that eliminates a chosen byte value (here, `0x00`) from an encoded byte stream.

**CR**: Coding Rate. LoRa forward-error-correction ratio: 4/5, 4/6, 4/7, or 4/8.

**FLRC**: Fast Long Range Communication. A higher-throughput modulation supported by SX128x and LR2021.

**LDRO**: Low Data Rate Optimization. A LoRa setting required at high SF and low BW to tolerate clock drift during long symbol times.

**LoRa**: Long Range. A chirp-spread-spectrum modulation technique from Semtech.

**LR-FHSS**: Long Range Frequency-Hopping Spread Spectrum. A Semtech modulation for low-throughput satellite or very-long-range use.

**Outstanding tag**: A tag for which the host has sent a command but not yet received its final response.

**SF**: Spreading Factor. LoRa symbol-duration parameter (5–12), trading throughput for range.

**Tag**: The 16-bit correlation ID in the frame header, used to match responses to commands.
