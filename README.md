# STPM32_UART

**English** | [Türkçe](README.tr.md)

UART driver for the **ST STPM32** single-phase energy metering IC on **ESP32** (Arduino core / PlatformIO).

- RMS voltage and current, active / fundamental / reactive / apparent power, energy counters, line frequency
- Automatic baud rate negotiation (survives ESP32 resets while the STPM32 stays powered) and CRC-checked frames
- Configuration stored in NVS and re-applied in a safe order at every boot
- Calibration trim (CHV1 / CHC1) against a reference meter
- Voltage sag / swell and overcurrent detection routed to the **INT1** pin
- Thread-safe: every public call is protected by a FreeRTOS recursive mutex
- No dynamic allocation after `begin()`

> ⚠️ STPM32 boards are connected to **mains voltage**. Only work on them if you know what you are doing, and use isolation (isolated USB / supply) when connecting a computer to a live board.

## Hardware

| STPM32 | ESP32 |
| --- | --- |
| TXD | any RX-capable pin (e.g. GPIO16) |
| RXD | any TX-capable pin (e.g. GPIO17) |
| INT1 (optional) | any input pin |
| GND | GND |

The STPM32 selects UART mode when **SCS is HIGH while EN goes HIGH** (datasheet, "Communication peripheral"). INT1 is **active-high** and stays HIGH while an enabled event is latched in the status register.

Only the primary channel (V1 / C1) is used. The STPM32 has no tamper module, so `setTamperProtection()` always returns `false`.

## Installation

**Arduino IDE:** Library Manager → search for `STPM32_UART` → Install.

**PlatformIO** (`platformio.ini`):

```ini
lib_deps = mustafabagr/STPM32_UART
```

**Manual:** download the repository as ZIP and use *Sketch → Include Library → Add .ZIP Library*.

## Quick start

```cpp
#include <STPM32_UART.h>

HardwareSerial StpmSerial(1);
STPMClass stpm(StpmSerial);

void setup() {
  Serial.begin(115200);
  stpm.setScale(0.031049f, 0.001341f);   // your hardware's V/LSB and A/LSB at gain x16
  stpm.begin(16, 17);                     // RX pin, TX pin (target baud 9600 by default)
}

void loop() {
  StpmSnapshot s;
  if (stpm.readSnapshot(s)) {
    Serial.printf("%.2f V  %.4f A  %.2f W  %.2f Hz\n", s.vRms, s.iRms, s.pActive, s.lineHz);
  }
  delay(1000);
}
```

## Scaling

The conversion from register counts to volts and amperes depends on your voltage divider and current sensor. The defaults (`STPM_DEFAULT_V_LSB`, `STPM_DEFAULT_I_LSB_X16`) were measured on the reference design (810 kΩ divider, 500 µΩ shunt) and will not match other hardware.

1. Start with the defaults or values computed from the datasheet ("RMS values" section).
2. Apply a known voltage and current, read them with the library, and correct:
   `newLsb = oldLsb × trueValue / measuredValue`.
3. Pass the result to `setScale(vLsb, iLsbAtGain16)` **before** `begin()`.
4. For fine adjustment, use the chip's own trim (`setVoltageTrim()` / `setCurrentTrim()`); see the `Calibration` example.

The power LSB is derived as `16 × LSB_V × LSB_I`, and the current LSB follows the current gain automatically (`setCurrentGain()`).

## Examples

| Example | What it shows |
| --- | --- |
| `BasicReadout` | Snapshot of all measurements once per second, energy accumulation, link recovery |
| `Protection` | Sag / swell / overcurrent on INT1, reading and selectively clearing the status register |
| `Calibration` | Trimming voltage and current against a reference meter over the Serial Monitor |

## API overview

| Function | Description |
| --- | --- |
| `setScale(vLsb, iLsbAtGain16)` | Hardware scale factors; call before `begin()` |
| `begin(rx, tx, targetBaud = 9600, nvsNamespace = "stpm_cfg")` | Starts UART, loads the configuration from NVS, connects and applies it |
| `recoverLink()` | Re-establishes the link after errors. 0 = failed, 1 = OK, 2 = OK and the chip had been reset (configuration re-applied) |
| `readSnapshot(StpmSnapshot&)` | All measurements under one lock |
| `readVoltageAndCurrent(v, i)`, `readActivePower(p)`, `readApparentPower(s)` | Individual readings |
| `energyLsbWh()` | Wh per count of the energy registers |
| `setCurrentGain(code)` | 0: x2, 1: x4, 2: x8, 3: x16 (re-encodes the overcurrent threshold) |
| `setVoltageTrim(expected, measured)`, `setCurrentTrim(expected, measured)` | Chip calibration (stored in NVS) |
| `setVoltageProtection(enable, sagV, swellV)`, `setCurrentProtection(enable, maxA)` | Thresholds and INT1 mask (stored in NVS) |
| `getProtectionSettings(...)` | Current thresholds and enable state |
| `readStatus(sr1)`, `clearStatus(mask)` | Latched status register (`STPM_IRQ::*` bit masks) |
| `readLiveEvents(ev1)`, `clearSagSwellHistory()` | Live events and sag/swell history |
| `readRegister32(addr, &v)`, `writeRegister32(addr, v)` | Raw register access (register unions are in the header) |

### Notes

- **Values are raw.** No anti-creep, power factor or energy policy is applied; do that in your application.
- **Energy counters wrap** (mod 2³²). Accumulate the signed difference between readings (see `BasicReadout`) and read often enough at your maximum power.
- **NVS:** each `STPMClass` instance needs its own `nvsNamespace` if you drive more than one chip.
- **Protection sequence:** mask off → clear stale status → write threshold → wait → mask on. A condition that is already present when protection is enabled correctly asserts INT1.

## License

[MIT](LICENSE)
