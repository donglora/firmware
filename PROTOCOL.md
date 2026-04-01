# DongLoRa USB Protocol

DongLoRa communicates over USB CDC-ACM (virtual serial port).
No baud rate configuration needed.

**USB identifiers:** VID `1209`, PID `5741`

## Framing

Each message is [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing)-encoded
and terminated with a `0x00` sentinel byte:

```
[COBS-encoded bytes] [0x00]
```

COBS guarantees `0x00` never appears in the encoded data, so the
sentinel unambiguously marks frame boundaries. USB packets are
chunked at 64 bytes.

## Encoding

All integers are **fixed-size little-endian**. No variable-length
encoding. Every field has a known size at a known offset.

| Type | Size | Encoding |
|------|------|----------|
| `u8` | 1 | Raw byte |
| `i8` | 1 | Raw byte (two's complement) |
| `u16` | 2 | Little-endian |
| `i16` | 2 | Little-endian (two's complement) |
| `u32` | 4 | Little-endian |

## RadioConfig (13 bytes)

| Offset | Size | Field | Type | Valid range |
|--------|------|-------|------|-------------|
| 0 | 4 | `freq_hz` | u32 LE | 150,000,000 – 960,000,000 |
| 4 | 1 | `bw` | u8 | 0–9 (see bandwidth table) |
| 5 | 1 | `sf` | u8 | 5–12 |
| 6 | 1 | `cr` | u8 | 5–8 (coding rate denominator) |
| 7 | 2 | `sync_word` | u16 LE | any |
| 9 | 1 | `tx_power_dbm` | i8 | board-dependent, or `-128` for max |
| 10 | 2 | `preamble_len` | u16 LE | 6–65535, or `0` for default (16) |
| 12 | 1 | `cad` | u8 | 0 = disabled, non-zero = enabled |

Set `tx_power_dbm` to `-128` (0x80) to use the board's maximum
TX power. The firmware resolves this to the actual hardware max.

Set `preamble_len` to `0` to use the firmware default of 16 symbols.
Longer preambles improve reliability in mesh/noisy environments at
the cost of slightly longer air time.

Set `cad` to `1` (or any non-zero value) to enable Channel Activity
Detection (listen-before-talk) before each transmission. The radio
checks for channel activity and waits for a clear channel before
transmitting. Set to `0` to disable. Default: enabled.

## Commands (Host → Firmware)

Each command is a tag byte followed by fixed-size fields.

| Tag | Command | Payload |
|-----|---------|---------|
| 0 | Ping | — |
| 1 | GetConfig | — |
| 2 | SetConfig | RadioConfig (10 bytes) |
| 3 | StartRx | — |
| 4 | StopRx | — |
| 5 | Transmit | has_config (1) + [RadioConfig if 1] + len (u16 LE) + payload |
| 6 | DisplayOn | — |
| 7 | DisplayOff | — |
| 8 | GetMac | — |

## Responses (Firmware → Host)

| Tag | Response | Payload |
|-----|----------|---------|
| 0 | Pong | — |
| 1 | Config | RadioConfig (10 bytes) |
| 2 | RxPacket | rssi (i16 LE) + snr (i16 LE) + len (u16 LE) + payload |
| 3 | TxDone | — |
| 4 | Ok | — |
| 5 | Error | code (u8) |
| 6 | MacAddress | 6 raw bytes (board's MAC/device address) |

## Bandwidth

| Value | Bandwidth |
|-------|-----------|
| 0 | 7.8 kHz |
| 1 | 10.4 kHz |
| 2 | 15.6 kHz |
| 3 | 20.8 kHz |
| 4 | 31.25 kHz |
| 5 | 41.7 kHz |
| 6 | 62.5 kHz |
| 7 | 125 kHz |
| 8 | 250 kHz |
| 9 | 500 kHz |

## Error Codes

| Value | Error | Meaning |
|-------|-------|---------|
| 0 | InvalidConfig | Radio config validation failed |
| 1 | RadioBusy | Radio busy (e.g. RX restart failed) |
| 2 | TxTimeout | Transmission timed out |
| 3 | CrcError | CRC error on received packet |
| 4 | NotConfigured | SetConfig required first |
| 5 | NoDisplay | No display attached |

## Worked Example

Configure for 915 MHz, 125 kHz BW, SF7, CR 4/5, max TX power, default preamble, CAD on:

### 1. Build the bytes

```python
import struct
config = struct.pack("<IBBBHBHB",
    915_000_000,  # freq_hz
    7,            # bw (125 kHz)
    7,            # sf
    5,            # cr (4/5)
    0x1424,       # sync_word
    0x80,         # tx_power_dbm (-128 = max)
    0,            # preamble_len (0 = default 16)
    1,            # cad (1 = enabled)
)
command = b"\x02" + config  # tag 2 = SetConfig
```

Raw bytes: `02 C0 CA 89 36 07 07 05 24 14 80 00 00 01`

### 2. COBS-encode and send

COBS-encode the bytes, append `0x00`, write to USB serial port.
Read back the response, COBS-decode it. First byte `0x04` = Ok.

## Command/Response Discipline

### Solicited vs. unsolicited responses

Every response is either **solicited** (exactly one per command) or
**unsolicited** (arrives at any time):

| Response | Category |
|----------|----------|
| `RxPacket` | **Unsolicited** — arrives whenever a LoRa packet is received during an active RX session |
| All others | **Solicited** — exactly one per command, in the order commands were sent |

### Command → response mapping

Each command produces exactly one solicited response:

| Command | Success | Possible errors |
|---------|---------|-----------------|
| Ping | Pong | — |
| GetConfig | Config | Error(NotConfigured) |
| SetConfig | Ok | Error(InvalidConfig) |
| StartRx | Ok | Error(NotConfigured), Error(InvalidConfig) |
| StopRx | Ok | — |
| Transmit | TxDone | Error(NotConfigured), Error(InvalidConfig), Error(TxTimeout) |
| DisplayOn | Ok | Error(NoDisplay) |
| DisplayOff | Ok | Error(NoDisplay) |
| GetMac | MacAddress | — |

### One outstanding command

The host **must** wait for the solicited response to the current
command before sending the next command. Pipelining multiple commands
without waiting for responses produces **undefined response ordering**.

### Processing order

Commands are processed in FIFO order. The firmware never reorders
commands or drops them silently.

### Interleaving with RxPacket

Between sending a command and receiving its solicited response, the
host may receive zero or more unsolicited `RxPacket` frames.
Hosts **must skip** `RxPacket` frames (tag 2) when waiting for a
solicited response. Skipped `RxPacket` frames should be buffered,
not discarded, if the application needs received LoRa data.

### Locally-handled commands

`DisplayOn`, `DisplayOff`, and `GetMac` are handled immediately by
the USB task and respond without involving the radio. All other
commands are routed to the radio task and respond after radio
processing. The one-outstanding-command rule ensures these two
response paths never conflict.

## Received Packet Quality

The SX126x radio will demodulate and deliver packets that are
technically intact (LoRa CRC passed) but were received so close to the
noise floor that the payload may be corrupt. This is inherent to LoRa:
the radio's internal CRC is only 16 bits, and at very low SNR the false-
positive rate is non-trivial. The firmware delivers every packet the
radio accepts — it is the host's responsibility to assess quality.

### SNR demodulation floor

Each spreading factor has a theoretical minimum SNR for reliable
demodulation. Packets received below this floor are statistically
likely to contain errors even though the radio reported a valid CRC:

| SF | Min SNR (dB) |
|----|--------------|
| 5  | -2.5         |
| 6  | -5.0         |
| 7  | -7.5         |
| 8  | -10.0        |
| 9  | -12.5        |
| 10 | -15.0        |
| 11 | -17.5        |
| 12 | -20.0        |

The pattern is: **min_snr = -2.5 × (SF - 4)**.

### SNR validity range

The SX126x reports SNR as a signed value in the range **-32 to +32 dB**.
Values outside this range indicate a firmware or decoding error — treat
the packet as unreliable.

### Recommended quality grading

Hosts should classify each `RxPacket` before trusting the payload:

```
function grade_packet(snr, sf):
    # Step 1: reject impossible SNR values
    if snr < -32 or snr > 32:
        return INVALID        # bad metadata, discard or flag

    # Step 2: check against demodulation floor
    min_snr = -2.5 * (sf - 4)
    margin  = snr - min_snr    # dB above the floor

    if margin < 0:
        return UNRELIABLE      # below floor, likely corrupt
    if margin < 3:
        return MARGINAL        # near floor, may contain errors
    return GOOD                # comfortable margin
```

**What to do with each grade:**

- **GOOD** — process normally.
- **MARGINAL** — process, but prefer application-layer integrity
  checks (checksums, sequence numbers, known-format validation)
  before acting on the payload. Log for diagnostics.
- **UNRELIABLE** — log for diagnostics but do not trust the payload
  for routing decisions, command execution, or retransmission.
  Applications may still display the data with a warning.
- **INVALID** — discard. The SNR metadata itself is corrupt.

### Application-layer defenses

For safety-critical or command-and-control applications, do not rely
solely on the radio CRC. Add your own integrity checks:

- An application-layer CRC or hash over the payload
- Sequence numbers to detect duplicates and missing packets
- Known-format validation (magic bytes, expected lengths)
- Require acknowledgement before acting on commands

### Radio config impact

Wider bandwidth and lower spreading factor increase throughput but
reduce sensitivity, making low-SNR packets more common in marginal
links. If your application sees frequent MARGINAL/UNRELIABLE packets,
consider increasing SF or decreasing BW to improve link margin at the
cost of air time.

## Host Implementation Checklist

1. Open USB serial port (find by VID:PID `1209:5741`)
2. Implement COBS encode/decode (libraries exist for every language)
3. Pack commands with `struct.pack` or equivalent — all fields are fixed-size LE
4. Optionally send `GetMac`, read back `MacAddress` (useful with multiple dongles)
5. Send `SetConfig`, read back `Ok`
6. Send `StartRx`, read back `Ok`
7. Loop: read frames, decode `RxPacket` (rssi, snr, payload at known offsets)
8. To transmit: send `Transmit`, read back `TxDone`

See `examples/` for working Python implementations.
