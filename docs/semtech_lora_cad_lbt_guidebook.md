# Semtech LoRa CAD and LBT Guidebook

**How to use Channel Activity Detection (CAD), RSSI, and CAD-to-TX modes on Semtech LoRa radios**

Revision: 1.0
Prepared: 2026-04-27
Audience: embedded firmware engineers implementing carrier sense, channel assessment, or listen-before-talk behavior on Semtech LoRa transceivers.

---

## 1. Executive summary

Do not implement LoRa listen-before-talk (LBT) as a blind two-command sequence such as:

```text
SetCAD();
SetTx();     // or Send(), Radio.Send(), start TX, etc.
```

That is not a reliable LBT implementation. It either ignores the CAD result, races the radio state machine, or cancels the CAD decision before the host has observed it.

There are only two defensible implementation patterns:

1. **Host-controlled CAD then TX**

   Start CAD, wait for the `CadDone` indication, inspect whether `CadDetected` was asserted, clear the IRQ flags, optionally perform an RSSI or energy check, and only then transmit.

2. **Documented CAD_LBT / CAD exit-to-TX**

   On radio families and driver stacks that explicitly support a CAD exit mode that enters TX when CAD finds no activity, configure that mode, preload the TX payload, and start CAD. In this model, the CAD operation substitutes for the normal TX command.

The important distinction is this:

```text
Correct:    configure CAD exit mode = TX/LBT, preload payload, then SetCAD()
Incorrect:  SetCAD(), then immediately SetTx()/Send()
```

CAD is not a general-purpose RF energy detector. CAD answers a narrower question: whether the radio detected LoRa-like activity under its current LoRa and CAD configuration. For regulatory LBT or coexistence with non-LoRa interferers, CAD is often not sufficient by itself; combine it with RSSI or energy CCA when the applicable rule or product behavior requires that.

---

## 2. Scope and assumptions

This guide covers practical CAD and LBT sequencing for the following Semtech LoRa radio families:

- SX1272/SX1276/SX1277/SX1278/SX1279 style register-based LoRa radios
- SX1261/SX1262/SX1268 class command-based radios
- LR1110/LR1120/LR1121 / LR11xx family radios
- SX1280/SX1281 2.4 GHz LoRa radios
- LLCC68, at a high level, because it is SX126x-like but should be verified against the exact datasheet and driver in use

This document is not a regulatory certification opinion. For legal LBT compliance, use the current regional standard and your test lab's interpretation for threshold, observation time, channel bandwidth, timing between assessment and TX, and duty-cycle or dwell-time constraints.

---

## 3. Key definitions

### CAD: Channel Activity Detection

CAD is a LoRa modem feature that attempts to detect LoRa activity with less energy and latency than fully entering continuous receive. Its exact behavior and configurability vary by radio family.

In most LoRa use cases, CAD is used to decide whether there appears to be another LoRa signal on the configured channel before entering RX or TX.

### `CadDone`

`CadDone` means the CAD operation has finished. It does not by itself mean the channel is clear or busy.

### `CadDetected`

`CadDetected` means the radio detected activity according to its CAD algorithm and current configuration.

A useful firmware rule is:

```text
CadDone && CadDetected      -> LoRa activity was detected
CadDone && !CadDetected     -> CAD did not detect LoRa activity
```

The second case does **not** prove that the RF channel is free of all energy or interference.

### LBT: Listen Before Talk

LBT is a policy or regulatory behavior: assess a channel before transmitting, transmit only when the channel is considered clear, and back off or retry when it is busy.

CAD can be part of an LBT implementation, but CAD alone is not always a complete LBT implementation.

### CCA: Clear Channel Assessment

CCA is a broader channel-clear decision. It may be based on RSSI, energy detection, CAD, packet detection, or a combination of these.

### CAD exit modes

Some Semtech command-based radios let CAD automatically transition to another mode after the CAD decision:

- `CAD_ONLY`: perform CAD and return to standby
- `CAD_RX`: enter RX if CAD detects activity
- `CAD_LBT`, `CAD_TX`, or `CAD_EXIT_MODE_TX`: enter TX if CAD does **not** detect activity

Names vary by radio family and driver.

---

## 4. The two correct CAD/LBT patterns

## 4.1 Pattern A: host-controlled CAD then TX

This is the portable pattern and the safe default.

Use it on:

- SX127x-class radios
- SX1280/SX1281
- SX126x when you are using only the CAD behavior documented in the public SX1261/2 Rev. 2.2 datasheet
- Any driver stack where CAD-to-TX support is uncertain

Sequence:

```text
1. Configure LoRa PHY and packet settings.
2. Configure CAD settings, if the radio exposes them.
3. Clear stale CAD IRQ flags.
4. Start CAD.
5. Wait for CadDone.
6. Read IRQ/status.
7. Clear CAD IRQ flags.
8. If CadDetected is set:
      treat the channel as LoRa-busy;
      back off, retry later, or enter RX.
   Else:
      optionally perform RSSI/energy CCA;
      transmit only if the final CCA policy says clear.
```

Generic pseudocode:

```c
configure_lora_phy();
configure_packet_params();
configure_cad_params_if_available();

clear_irq(CAD_DONE | CAD_DETECTED);
start_cad();

wait_for_irq(CAD_DONE);
irq = get_irq_status();
clear_irq(CAD_DONE | CAD_DETECTED);

if (irq & CAD_DETECTED) {
    backoff_or_enter_rx();
} else {
    if (energy_cca_required()) {
        if (energy_is_clear()) {
            transmit();
        } else {
            backoff();
        }
    } else {
        transmit();
    }
}
```

## 4.2 Pattern B: documented CAD_LBT / CAD exit-to-TX

Use this only when the exact radio and driver document a CAD exit mode that enters TX after a clear CAD decision.

Sequence:

```text
1. Configure LoRa PHY.
2. Configure TX parameters.
3. Configure packet parameters.
4. Configure buffer base addresses, where applicable.
5. Write the TX payload into the radio buffer.
6. Configure CAD exit mode = TX/LBT.
7. Start CAD.
8. Wait for TX done, timeout, or CAD-related IRQs according to the driver.
```

Generic pseudocode:

```c
configure_lora_phy();
configure_tx_params();
configure_packet_params();
configure_buffer_base_addresses();
write_tx_payload(payload, len);

set_cad_params(
    cad_symbols,
    cad_det_peak,
    cad_det_min,
    CAD_EXIT_MODE_TX,   // name depends on radio/driver
    tx_timeout
);

start_cad();            // this substitutes for transmit()

wait_for_completion_irq();
handle_tx_done_timeout_or_cad_status();
```

Do not call the normal transmit command immediately after starting CAD in this mode. The CAD operation is the transmit gate.

---

## 5. Device-family decision matrix

| Radio family                             | Is blind `CAD; SEND` valid? |                                                                                             Documented CAD-to-TX? | Correct method                                                                                                |
| ---------------------------------------- | --------------------------: | ----------------------------------------------------------------------------------------------------------------: | ------------------------------------------------------------------------------------------------------------- |
| SX1272/SX1276/SX1277/SX1278/SX1279 style |                          No |                                                       No documented CAD_LBT in the SX1276/77/78/79 register model | Host-controlled CAD, wait `CadDone`, inspect `CadDetected`, then TX only if clear                             |
| SX1261/SX1262/SX1268 class               |                          No | Public SX1261/2 Rev. 2.2 datasheet documents `CAD_ONLY` and `CAD_RX`; Semtech SDK also documents `SX126X_CAD_LBT` | Use host-controlled CAD for datasheet-portable code; use SDK `CAD_LBT` only when your exact stack supports it |
| LR1110/LR1120/LR1121 / LR11xx            |                          No |                                          LR1121 manual documents `CAD_LBT`; LR11xx SDK documents TX-exit CAD mode | Use documented CAD exit-to-TX with payload preloaded, or host-controlled CAD                                  |
| SX1280/SX1281                            |                          No |                                                                        No CAD_LBT in the SX1280/81 datasheet flow | Host-controlled CAD, wait `CadDone`, inspect `CadDetected`, then TX only if clear                             |
| LLCC68                                   |                          No |                                                                                     Verify exact datasheet/driver | Treat as SX126x-like, but confirm supported CAD exit modes before using CAD-to-TX                             |

---

## 6. CAD is not the same as RF energy detection

A common implementation error is treating this condition as a complete channel-clear proof:

```text
CadDone && !CadDetected
```

That condition only means CAD did not detect LoRa activity according to the radio's current CAD and LoRa configuration.

It does not guarantee that:

- no other RF signal is present;
- no FSK, BLE, Wi-Fi leakage, CW interferer, jammer, or broadband noise is present;
- another LoRa signal using a different configuration is absent;
- the channel satisfies a regional energy-based LBT rule;
- the channel will remain clear by the time TX begins.

A robust CCA policy is often:

```text
busy if CAD detects LoRa activity
busy if RSSI/energy is above the chosen threshold
clear only if both CAD and energy checks pass
```

For purely LoRa-network collision avoidance, CAD alone may be acceptable in some products. For regulatory LBT, do not assume CAD alone is enough unless your standard, certification plan, and test lab agree.

---

## 7. SX127x guide

This section applies to SX1272/SX1276/SX1277/SX1278/SX1279 style register-based LoRa radios. The exact register map differs somewhat by part, but the CAD control model is the same general style.

## 7.1 What the radio supports

SX127x-class radios expose CAD as a radio operating mode, separate from TX and RX. In the SX1276/77/78/79 datasheet model, TX, RX single, RX continuous, and CAD are different operating modes selected through the operation-mode register.

CAD completion is reported through CAD-related IRQ flags:

- `CadDone`: CAD operation finished
- `CadDetected`: CAD found activity

The CAD operation returns the radio to standby after completion.

There is no documented hardware CAD-to-TX exit mode in the SX1276/77/78/79 register model. Therefore, LBT must be host-controlled.

## 7.2 Correct SX127x host-controlled CAD/LBT sequence

```c
// SX127x-style pseudocode.
// Symbol names are illustrative. Use the definitions from your driver.

// 1. Configure LoRa mode and modem parameters.
set_lora_mode();
set_frequency(freq_hz);
set_spreading_factor(sf);
set_bandwidth(bw);
set_coding_rate(cr);
set_preamble_length(preamble_len);
set_payload_params(payload_len, crc_on, explicit_header);

// 2. Optionally preload TX FIFO to reduce the gap between clear CAD and TX.
load_tx_fifo(payload, payload_len);

// 3. Clear stale CAD IRQ flags.
write_reg(REG_IRQ_FLAGS, IRQ_CAD_DONE | IRQ_CAD_DETECTED);

// 4. Start CAD.
write_reg(REG_OP_MODE, MODE_LORA | MODE_CAD);

// 5. Wait for CAD completion.
while ((read_reg(REG_IRQ_FLAGS) & IRQ_CAD_DONE) == 0) {
    // Poll, sleep until DIO interrupt, or service other tasks.
}

uint8_t irq = read_reg(REG_IRQ_FLAGS);

// 6. Clear CAD flags by writing 1s to those bits.
write_reg(REG_IRQ_FLAGS, IRQ_CAD_DONE | IRQ_CAD_DETECTED);

// 7. Make the LBT decision.
if (irq & IRQ_CAD_DETECTED) {
    // LoRa activity was detected.
    schedule_random_backoff();
    // or enter RX to receive the packet, depending on your MAC.
} else {
    if (rssi_cca_required() && !rssi_energy_clear()) {
        schedule_random_backoff();
    } else {
        // FIFO was preloaded above; otherwise load it here.
        write_reg(REG_OP_MODE, MODE_LORA | MODE_TX);
    }
}
```

## 7.3 SX127x notes

- Clear stale CAD IRQ flags before starting a new CAD operation.
- Do not read `CadDetected` before `CadDone` and treat it as final.
- Do not issue TX until CAD has completed and the host has evaluated the result.
- Preloading the FIFO before CAD can reduce the time between clear CAD and TX, but it does not remove the need to wait for `CadDone`.
- If your LBT policy must detect non-LoRa interferers, add RSSI or energy CCA.

---

## 8. SX126x guide

This section applies to SX1261/SX1262/SX1268 style command-based radios.

## 8.1 Datasheet-portable behavior

The public SX1261/2 Rev. 2.2 datasheet documents CAD as a LoRa-only operation that searches for LoRa preamble activity. CAD reports:

- `CADdone`
- `CadDetected`, if a valid signal was detected

`SetCadParams` includes parameters such as:

- number of CAD symbols
- detection peak threshold
- detection minimum threshold
- CAD exit mode
- CAD timeout

The public Rev. 2.2 datasheet table documents at least these exit modes:

- `CAD_ONLY`: perform CAD and return to standby
- `CAD_RX`: if activity is detected, enter RX

For code that must be portable to the behavior described in that datasheet, use host-controlled CAD or `CAD_RX`, not an assumed CAD-to-TX path.

## 8.2 SX126x host-controlled LBT sequence

```c
// SX126x-style pseudocode.
// Function names vary across drivers.

wait_while_busy();
SetStandby(STDBY_RC);

wait_while_busy();
SetPacketType(PACKET_TYPE_LORA);

wait_while_busy();
SetRfFrequency(freq_hz);

wait_while_busy();
SetModulationParams(sf, bw, cr, ldro);

wait_while_busy();
SetPacketParams(preamble_len,
                header_type,
                payload_len,
                crc_on,
                iq_inverted);

wait_while_busy();
SetTxParams(tx_power_dbm, ramp_time);

wait_while_busy();
SetBufferBaseAddress(tx_base, rx_base);

wait_while_busy();
WriteBuffer(tx_base, payload, payload_len);  // optional preload

wait_while_busy();
SetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_DETECTED | IRQ_TX_DONE | IRQ_TIMEOUT,
                dio1_mask,
                dio2_mask,
                dio3_mask);

wait_while_busy();
ClearIrqStatus(IRQ_CAD_DONE | IRQ_CAD_DETECTED);

wait_while_busy();
SetCadParams(cad_symbols,
             cad_det_peak,
             cad_det_min,
             CAD_ONLY,
             0);

wait_while_busy();
SetCAD();

wait_for_irq(IRQ_CAD_DONE);

wait_while_busy();
uint16_t irq = GetIrqStatus();

wait_while_busy();
ClearIrqStatus(IRQ_CAD_DONE | IRQ_CAD_DETECTED);

if (irq & IRQ_CAD_DETECTED) {
    backoff_or_enter_rx();
} else if (energy_cca_required() && !energy_is_clear()) {
    backoff();
} else {
    wait_while_busy();
    SetTx(tx_timeout);
}
```

## 8.3 SX126x SDK `CAD_LBT` behavior

Semtech's SWSD003 SX126x CAD example documents `SX126X_CAD_LBT`. In that mode, if no activity is detected, the chip goes to TX. The same example states that this mode is a substitute for TX and that payload data should be preloaded before entering the CAD LBT mode.

Use this only when your exact driver, chip, and integration expose and support it.

```c
// SX126x SDK-style CAD_LBT pseudocode.

wait_while_busy();
SetStandby(STDBY_RC);

wait_while_busy();
SetPacketType(PACKET_TYPE_LORA);

wait_while_busy();
SetRfFrequency(freq_hz);

wait_while_busy();
SetModulationParams(sf, bw, cr, ldro);

wait_while_busy();
SetPacketParams(preamble_len,
                header_type,
                payload_len,
                crc_on,
                iq_inverted);

wait_while_busy();
SetTxParams(tx_power_dbm, ramp_time);

wait_while_busy();
SetBufferBaseAddress(tx_base, rx_base);

// Required before CAD_LBT, because CAD_LBT substitutes for SetTx().
wait_while_busy();
WriteBuffer(tx_base, payload, payload_len);

wait_while_busy();
SetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_DETECTED | IRQ_TX_DONE | IRQ_TIMEOUT,
                dio1_mask,
                dio2_mask,
                dio3_mask);

wait_while_busy();
ClearIrqStatus(IRQ_CAD_DONE | IRQ_CAD_DETECTED | IRQ_TX_DONE | IRQ_TIMEOUT);

wait_while_busy();
SetCadParams(cad_symbols,
             cad_det_peak,
             cad_det_min,
             SX126X_CAD_LBT,
             tx_timeout);

wait_while_busy();
SetCAD();       // This is the gated TX attempt.

// Do not immediately call SetTx() here.
// Wait for the completion indication defined by your driver.
```

## 8.4 SX126x notes

- Always respect the `BUSY` line before sending commands.
- Treat `cadDetPeak` and `cadDetMin` as parameters requiring validation for your SF, bandwidth, packet timing, and false-detection tolerance.
- If using `CAD_LBT`, preload the payload and TX configuration before starting CAD.
- If using only the public Rev. 2.2 datasheet behavior, do not assume `CAD_LBT` exists.
- For regulatory LBT, add RSSI or energy assessment when required.

---

## 9. LR11xx / LR1121 guide

This section applies to LR1110/LR1120/LR1121 family radios at a high level, with the strongest direct documentation in this guide coming from the LR1121 user manual and the Semtech LR11xx SDK example.

## 9.1 What the radio supports

The LR1121 manual documents CAD as a LoRa operation that reports:

- `CADdone`
- `CadDetected`, when valid activity is found

Its CAD parameters include:

- number of symbols
- detection peak threshold
- detection minimum threshold
- CAD exit mode
- timeout

The LR1121 manual documents these CAD exit modes:

- `CAD_ONLY`: after CAD, return to standby
- `CAD_RX`: if activity is detected, stay in RX until packet or timeout
- `CAD_LBT`: if no activity is detected, enter TX mode

For `CAD_LBT`, the timeout is interpreted as the TX timeout.

Semtech's LR11xx SDK CAD example similarly documents a TX-exit CAD mode and states that the payload should be preloaded because this mode substitutes for TX.

## 9.2 LR11xx CAD_LBT sequence

```c
// LR11xx/LR1121-style pseudocode.
// Function and enum names vary by driver.

SetPacketType(PACKET_TYPE_LORA);
SetRfFrequency(freq_hz);
SetModulationParams(sf, bw, cr, ldro);
SetPacketParams(preamble_len,
                header_type,
                payload_len,
                crc_on,
                iq_inverted);

SetTxParams(tx_power_dbm, ramp_time);
SetBufferBaseAddress(tx_base, rx_base);
WriteBuffer(tx_base, payload, payload_len);   // Required before CAD-to-TX

SetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_DETECTED | IRQ_TX_DONE | IRQ_TIMEOUT,
                dio1_mask,
                dio2_mask,
                dio3_mask);

ClearIrqStatus(IRQ_CAD_DONE | IRQ_CAD_DETECTED | IRQ_TX_DONE | IRQ_TIMEOUT);

SetCadParams(cad_symbols,
             cad_det_peak,
             cad_det_min,
             CAD_LBT,
             tx_timeout);

SetCAD();     // If CAD is clear, the radio enters TX.

wait_for_completion_irq();
```

## 9.3 LR11xx notes

- Do not use `SetCAD(); SetTx();` as a substitute for `CAD_LBT`.
- Preload the payload before starting CAD_LBT.
- Verify the exact enum names and behavior for LR1110/LR1120/LR1121 in the driver version you ship.
- If certification matters, verify the behavior against the exact firmware, radio revision, and regional rules.

---

## 10. SX1280/SX1281 guide

This section applies to SX1280/SX1281 2.4 GHz LoRa radios.

## 10.1 What the radio supports

The SX1280/81 datasheet describes `SetCAD()` as a LoRa operation that searches for a LoRa signal. It returns to standby after the search and reports:

- `CadDone`
- `CadDetected`, if a valid signal was found

`SetCadParams()` for SX1280/SX1281 selects the number of CAD symbols. Supported values include 1, 2, 4, 8, and 16 symbols. The datasheet warns that using very small symbol counts, especially 1 or 2 symbols, increases false-detection risk.

The SX1280/81 datasheet flow does not document a CAD-to-TX exit mode analogous to LR1121 `CAD_LBT`.

## 10.2 Correct SX128x host-controlled sequence

```c
// SX1280/SX1281-style pseudocode.

SetPacketType(PACKET_TYPE_LORA);
SetRfFrequency(freq_hz);
SetModulationParams(sf, bw, cr);
SetPacketParams(preamble_len,
                header_type,
                payload_len,
                crc_on,
                iq_inverted);
SetTxParams(tx_power_dbm, ramp_time);

SetBufferBaseAddress(tx_base, rx_base);
WriteBuffer(tx_base, payload, payload_len);   // optional preload

SetCadParams(LORA_CAD_04_SYMBOLS);            // choose based on robustness/latency

SetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_DETECTED | IRQ_TX_DONE | IRQ_TIMEOUT,
                dio1_mask,
                dio2_mask,
                dio3_mask);

ClearIrqStatus(IRQ_CAD_DONE | IRQ_CAD_DETECTED);

SetCAD();

wait_for_irq(IRQ_CAD_DONE);
irq = GetIrqStatus();
ClearIrqStatus(IRQ_CAD_DONE | IRQ_CAD_DETECTED);

if (irq & IRQ_CAD_DETECTED) {
    backoff_or_enter_rx();
} else if (energy_cca_required() && !energy_is_clear()) {
    backoff();
} else {
    SetTx(tx_timeout);
}
```

## 10.3 SX128x notes

- Use host-controlled CAD for LBT.
- Avoid relying on one-symbol or two-symbol CAD unless your testing shows the false-detection behavior is acceptable.
- Add an energy check when non-LoRa interferers or regulatory LBT matter.

---

## 11. LLCC68 note

LLCC68 is SX126x-like in its command structure, but this guide does not assert a universal LLCC68 CAD-to-TX behavior. Verify the exact LLCC68 datasheet and driver you are using.

Safe default:

```text
Use host-controlled CAD:
    SetCadParams(...)
    SetCAD()
    wait CadDone
    inspect CadDetected
    TX only if clear
```

Use CAD-to-TX only if your exact LLCC68 documentation and driver explicitly define it.

---

## 12. RSSI plus CAD

## 12.1 Why RSSI may still be needed

CAD is useful because LoRa signals can be difficult to assess with simple RSSI, especially when they are near or below the apparent receiver noise floor. However, CAD is not an energy detector for every possible interferer.

Use RSSI or energy CCA when you need to detect:

- non-LoRa transmitters;
- strong out-of-network interferers;
- continuous wave or broadband noise;
- signals using configurations your CAD settings may not detect;
- energy-based regulatory occupancy.

## 12.2 Conservative CCA decision

A conservative channel decision is:

```c
bool channel_busy = false;

if (cad_detected) {
    channel_busy = true;
}

if (energy_cca_required() && rssi_or_energy_above_threshold()) {
    channel_busy = true;
}

if (channel_busy) {
    backoff();
} else {
    transmit();
}
```

Or, as a truth table:

| CAD result   | RSSI/energy result  | Conservative decision                         |
| ------------ | ------------------- | --------------------------------------------- |
| CAD detected | Any                 | Busy                                          |
| CAD clear    | Energy high         | Busy                                          |
| CAD clear    | Energy low          | Clear                                         |
| CAD clear    | Energy not measured | Clear only if your policy allows CAD-only CCA |

## 12.3 RSSI during CAD

Semtech application note AN1200.21 describes reading RSSI during CAD on SX127x/SX1272-era radios. The idea is to start CAD, wait for the appropriate CAD startup and symbol timing point, read RSSI while the radio is effectively observing the channel, then wait for CAD completion.

This can combine a LoRa CAD decision and an RSSI sample with relatively low energy cost. Treat the timing and thresholds as implementation details that must be validated for the target radio, driver, bandwidth, spreading factor, and regional rule.

---

## 13. CAD parameter selection

## 13.1 CAD symbol count

Increasing the number of CAD symbols generally improves detection confidence and reduces false decisions, at the cost of more time and energy.

Lower symbol counts reduce latency and power but increase risk:

- missed detection;
- false detection;
- less reliable behavior at low SNR;
- poorer LBT decisions under marginal conditions.

For radios that expose symbol count, validate the chosen value under realistic network traffic and interference.

## 13.2 Detection thresholds

SX126x and LR11xx-style radios expose CAD detection threshold parameters such as detection peak and detection minimum.

Do not blindly copy threshold values across products. The correct operating point can depend on:

- spreading factor;
- bandwidth;
- expected SNR;
- preamble length;
- traffic timing;
- antenna and RF front-end behavior;
- false-alarm tolerance;
- missed-detection tolerance;
- whether CAD is used for RX wakeup, collision avoidance, or regulatory LBT support.

## 13.3 Time gap between CAD clear and TX

Host-controlled CAD always has a gap between CAD completion and transmit start. During that gap, another node may begin transmitting. Reduce the gap by:

- preloading the TX buffer before CAD;
- preconfiguring TX parameters before CAD;
- servicing the CAD IRQ promptly;
- using a documented CAD-to-TX mode where available;
- applying random backoff in multi-node networks.

Even CAD_LBT does not make collisions impossible. It only reduces the assessment-to-TX gap and lets the radio perform the transition according to its documented state machine.

---

## 14. Recommended implementation checklist

Before CAD:

- [ ] Put the radio in the required standby or command-ready state.
- [ ] Configure packet type to LoRa.
- [ ] Configure RF frequency.
- [ ] Configure LoRa modulation parameters: spreading factor, bandwidth, coding rate, and low-data-rate optimization where applicable.
- [ ] Configure packet parameters needed for the subsequent RX/TX path.
- [ ] Configure TX power, ramp, and PA settings if TX may follow CAD.
- [ ] Configure buffer base addresses where applicable.
- [ ] Preload TX payload if using CAD_LBT or if minimizing CAD-to-TX delay.
- [ ] Configure CAD parameters.
- [ ] Configure IRQ routing for `CadDone`, `CadDetected`, `TxDone`, and timeout as needed.
- [ ] Clear stale IRQ flags.
- [ ] Respect the `BUSY` line on command-based radios.

During CAD:

- [ ] Do not issue TX until CAD is complete, unless the radio is in a documented CAD_LBT mode.
- [ ] Wait for `CadDone` or the driver's completion callback.
- [ ] Do not treat intermediate IRQ state as the final CAD decision.

After CAD:

- [ ] Read IRQ/status.
- [ ] Clear CAD IRQ flags.
- [ ] If `CadDetected`, back off or enter RX.
- [ ] If not detected, optionally perform RSSI/energy CCA.
- [ ] Transmit only when your final CCA/LBT policy says clear.
- [ ] Wait for `TxDone` or timeout.
- [ ] Clear TX-related IRQs.

For CAD_LBT:

- [ ] Confirm the exact chip and driver support TX-exit CAD.
- [ ] Preload the payload before starting CAD.
- [ ] Configure the CAD exit timeout as the TX timeout if required by the radio.
- [ ] Do not immediately call `SetTx()` after `SetCAD()`.
- [ ] Handle the completion IRQs exactly as specified by the driver.

---

## 15. Common incorrect implementations

## 15.1 Blind CAD then send

Incorrect:

```c
SetCAD();
Send(payload);
```

Why it is wrong:

- It does not wait for `CadDone`.
- It does not inspect `CadDetected`.
- It may cancel or override CAD.
- It does not implement channel assessment.

## 15.2 Polling `CadDetected` without waiting for `CadDone`

Incorrect:

```c
SetCAD();
if (!(GetIrqStatus() & IRQ_CAD_DETECTED)) {
    SetTx(timeout);
}
```

Why it is wrong:

- `CadDetected` is meaningful as a CAD result only after CAD has completed.
- Before `CadDone`, absence of `CadDetected` is not a final clear-channel indication.

## 15.3 Using CAD as a universal energy detector

Incorrect assumption:

```text
No CadDetected means no RF energy is present.
```

Why it is wrong:

- CAD is LoRa-specific and configuration-dependent.
- Non-LoRa interferers may not be detected by CAD.
- Regulatory LBT may require energy measurement rather than LoRa activity detection.

## 15.4 Using CAD_LBT without preloading the payload

Incorrect:

```c
SetCadParams(..., CAD_LBT, timeout);
SetCAD();
WriteBuffer(payload);     // too late
```

Why it is wrong:

- In CAD-to-TX modes, CAD is the gated transmit attempt.
- The radio may enter TX immediately after a clear CAD decision.
- The payload and TX parameters must already be ready.

## 15.5 Assuming SX126x datasheet and SDK expose identical CAD modes

Incorrect assumption:

```text
Every SX126x stack has CAD_LBT because a Semtech SDK example mentions it.
```

Why it is risky:

- The public SX1261/2 Rev. 2.2 datasheet table documents `CAD_ONLY` and `CAD_RX`.
- Semtech's SDK documents `SX126X_CAD_LBT`.
- Your product should verify the exact driver, silicon, and documentation you rely on.

---

## 16. Testing and validation plan

## 16.1 Functional tests

Test at minimum:

- CAD clear channel -> TX occurs
- CAD busy channel -> TX is suppressed
- CAD busy channel with random backoff -> retry later
- stale CAD IRQ flags cleared before CAD start
- `CadDone` without `CadDetected` handled correctly
- `CadDone` with `CadDetected` handled correctly
- TX timeout handled correctly
- TX done handled correctly
- BUSY-line handling on command-based radios

## 16.2 RF tests

Test with:

- same SF/BW LoRa interferer;
- different SF or bandwidth LoRa interferer;
- short preamble packets;
- long preamble packets;
- packets already in progress;
- weak signals near sensitivity;
- strong adjacent-channel signals;
- non-LoRa interferers;
- noise or CW interference;
- multiple nodes attempting CAD/LBT simultaneously.

## 16.3 Parameter sweep

Sweep:

- CAD symbol count;
- `cadDetPeak` and `cadDetMin`, where available;
- RSSI threshold;
- RSSI observation time;
- random backoff window;
- time between CAD clear and TX;
- payload length and preamble length.

Measure:

- missed detection rate;
- false detection rate;
- collision rate;
- channel access latency;
- energy consumption;
- throughput;
- fairness between nodes.

## 16.4 Regulatory validation

For regulatory LBT, record:

- applicable region and rule version;
- required channel observation time;
- required energy threshold;
- bandwidth over which energy is assessed;
- maximum allowed delay between clear assessment and TX;
- random backoff requirements;
- maximum channel occupancy time;
- test-lab interpretation;
- final firmware version and radio driver commit.

---

## 17. Practical decision tree

```text
Do you need to transmit only if LoRa activity is absent?
    Yes:
        Does your exact radio/driver document CAD_LBT or CAD exit-to-TX?
            Yes:
                Preload TX payload.
                Configure CAD exit mode = TX/LBT.
                Start CAD.
                Do not call SetTx immediately after SetCAD.
            No:
                Start CAD.
                Wait CadDone.
                If CadDetected, back off.
                If not CadDetected, transmit.

Do you need to satisfy energy-based LBT or detect non-LoRa interferers?
    Yes:
        Add RSSI/energy CCA.
        Treat channel as busy if either CAD detects activity or energy is high.
    No:
        CAD-only may be acceptable for LoRa collision avoidance, after testing.
```

---

## 18. Final rules of thumb

1. **Never treat `SetCAD(); SetTx();` as LBT.**
2. **Wait for `CadDone` before using the CAD result.**
3. **Treat `CadDetected` as LoRa activity, not generic RF energy.**
4. **Use RSSI/energy CCA when non-LoRa interference or regulation matters.**
5. **Preload the payload before using CAD_LBT or CAD exit-to-TX.**
6. **Use CAD_LBT only when the exact chip and driver document it.**
7. **Tune CAD thresholds and symbol counts with RF testing, not assumptions.**
8. **Clear stale IRQs before and after CAD.**
9. **Respect the command-radio `BUSY` line.**
10. **Validate timing from clear assessment to TX.**

---

## Appendix A: Diffs from the initial informal answer

The initial informal answer was directionally correct but needed tightening. The final guidebook makes these changes:

- Replaces the broad statement "Semtech radios do not support single CAD then SEND" with the precise rule: blind `CAD; SEND` is wrong, but documented CAD exit-to-TX modes exist on some families and stacks.
- Separates host-controlled CAD from hardware/driver CAD_LBT.
- Clarifies that SX1261/2 Rev. 2.2 datasheet documentation and Semtech SDK examples are not identical: the datasheet table documents `CAD_ONLY` and `CAD_RX`; the SDK documents `SX126X_CAD_LBT`.
- Clarifies that LR1121 explicitly documents `CAD_LBT`, and LR11xx SDK examples document TX-exit CAD modes.
- Removes the implication that configuring a sync word is part of CAD detection itself. Configure packet and sync settings for the RX/TX path, but CAD is not a full packet validation step.
- Strengthens the distinction between CAD and RSSI/energy CCA.
- Adds implementation checklists, wrong-pattern examples, and validation tests.

---

## Appendix B: References

1. [Semtech SX1276/77/78/79 datasheet PDF mirror](https://cdn.sparkfun.com/assets/7/7/3/2/2/SX1276_Datasheet.pdf)
2. [Semtech SX1261/2 Rev. 2.2 datasheet PDF mirror](https://resource.heltec.cn/download/WiFi_LoRa_32_V4/datasheet/SX1261_2%20V2-2.pdf)
3. [Semtech SWSD003 SX126x CAD example README](https://github.com/Lora-net/SWSD003/blob/master/sx126x/apps/cad/README.md)
4. [Semtech LR1121 user manual mirror](https://manualzz.com/doc/89606071/semtech-lr1121-lora%C2%AE-transceiver-user-manual)
5. [Semtech SWSD003 LR11xx CAD example README](https://github.com/Lora-net/SWSD003/blob/master/lr11xx/apps/cad/README.md)
6. [Semtech SX1280/SX1281 Rev. 3.2 datasheet PDF mirror](https://media.digikey.com/pdf/Data%20Sheets/Semtech%20PDFs/SX1280-81_Rev3.2_Mar2020.pdf)
7. [Semtech AN1200.21, Reading Channel RSSI During a CAD](https://www.dessy.ru/include/images/ware/pdf/a/AN1200.21_STD.pdf)
8. [Semtech LLCC68 datasheet section mirror: SetCadParams](https://manualzz.com/doc/o/15k4mx/semtech-llcc68-data-sheet-lora%C2%AE-transceiver-datasheet-13.4.7-setcadparams)

---

## Appendix C: Minimal one-page implementation recipe

Use this when you need the shortest practical firmware rule.

```text
For SX127x/SX128x or portable SX126x code:
    configure LoRa PHY and TX packet
    optionally preload TX buffer
    clear CAD IRQs
    start CAD
    wait CadDone
    read IRQs
    clear CAD IRQs
    if CadDetected:
        backoff or RX
    else:
        if RSSI/energy CCA required and energy is high:
            backoff
        else:
            TX

For LR1121/LR11xx or supported SX126x CAD_LBT:
    confirm exact driver/chip supports CAD exit-to-TX
    configure LoRa PHY and TX packet
    preload TX payload
    configure CAD exit mode = TX/LBT
    start CAD
    do not issue SetTx immediately
    let radio enter TX only if CAD is clear
```
