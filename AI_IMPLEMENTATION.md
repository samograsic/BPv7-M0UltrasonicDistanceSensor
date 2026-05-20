# AI Implementation Notes — BPv7-M0UltrasonicDistanceSensor

This document is written for Claude Code (or any AI coding assistant) picking up
this project.  Read `CLAUDE.md` first for architecture and hardware overview.

---

## 1. Project status at handoff

All application source files are implemented:

| File | Status | Notes |
|------|--------|-------|
| `platformio.ini` | ✅ complete | |
| `src/config.h` | ✅ complete | Adjust NODE_NUMBER, LoRa params |
| `src/main.cpp` | ✅ complete | Full state machine |
| `src/rtc_manager.h/.cpp` | ✅ complete | RTCZero + GPS + epoch + alarm |
| `src/sensor.h/.cpp` | ✅ complete | Sonar driver, stats, payload |

The **TinyDTN library** (`BundleManager`, `LoRaCL`, `BPv7Codec`, `BPEchoService`,
`EndpointID`, `Bundle`) is **shared from the sibling BPv7-BaseStation project**
via `lib_extra_dirs = ../BPv7-BaseStation/lib` in `platformio.ini`.

---

## 2. First build checklist

```bash
cd c:/Users/<user>/Documents/PlatformIO/Projects/BPv7-M0UltrasonicDistanceSensor
pio run
```

Expected issues and fixes:

### 2a. `lib_extra_dirs` path not found
If the base station project is not at `../BPv7-BaseStation`, update the path in
`platformio.ini`:
```ini
lib_extra_dirs = /absolute/path/to/BPv7-BaseStation/lib
```

### 2b. `SAMD51_RTC` compiled for M0
The M4 base station has a `lib/SAMD51_RTC/` directory.  PlatformIO's `chain+` LDF
should NOT pull it in for M0, because nothing in the sensor code includes it.
If it does compile (causing SAMD51 register errors), add to `platformio.ini`:
```ini
lib_ignore = SAMD51_RTC
```

### 2c. `LTE_CL.cpp` / `DTNexService.cpp` pulled in
Similarly, if LTE_CL or DTNexService get compiled, add:
```ini
lib_ignore = LTE_CL, DTNexService, DebugService
```
These files are only included when the corresponding `.h` files are included; with
`chain+` mode they should be silent.

### 2d. `cpu_utils.h` not found (DTNexService dependency)
If DTNexService.cpp is pulled in and fails on `#include <cpu_utils.h>`, add
`DTNexService` to `lib_ignore`.

### 2e. NeoGPS not found
The sensor uses NeoGPS (same as base station).  It is listed in `lib_deps`:
```
SlashDevin/NeoGPS@^4.2.9
```
If `pio run` can't find it, try:
```bash
pio lib install "SlashDevin/NeoGPS"
```

---

## 3. M0 vs M4 — key differences already handled

| Concern | M4 approach | M0 approach (implemented) |
|---------|-------------|---------------------------|
| RTC | custom `SAMD51_RTC.cpp` | `RTCZero` (Arduino library) |
| Sleep | `RTC->MODE2.CTRLA` direct register write | `rtc_.standbyMode()` |
| Alarm ISR | `RTC_Handler()` with APB gating fix | `RTCZero.attachInterrupt()` |
| GPS enable | A2 | pin 11 |
| LoRa pins | CS=2, RST=A1, DIO0=3 | CS=8, RST=4, DIO0=3 |
| Battery ADC | custom SAMD51 ADC code | `analogRead(A7)` (Feather built-in) |
| Temperature | SAMD51 internal sensor | SAMD21 internal sensor (same method) |
| systemDelay() | `power_management.cpp` | `delay()` + `Watchdog.reset()` |

---

## 4. Sonar sensor details

**I2CXL-MaxSonar-EZ** series (MB1202, MB1212, MB1222, MB1232, MB1242, MB1242):

- I2C 7-bit address: `0x70` (default; pin-selectable on some models)
- Ranging command: write `0x51` → wait 100 ms → read 2 bytes (big-endian cm)
- Valid range: 20–765 cm (model dependent)
- Current draw: ~2 mA active, < 1 µA if powered down

The sensor is always powered (3.3 V) in this implementation.  To save power when
sleeping (adds ~2 mA × 9 min × 144 cycles/day ≈ 26 mAh/day), control it with a
MOSFET or load switch:
```cpp
#define SONAR_POWER_PIN  5   // HIGH = on, LOW = off
// In setup():  pinMode(SONAR_POWER_PIN, OUTPUT);
// Before measure(): digitalWrite(SONAR_POWER_PIN, HIGH); delay(50);
// After measure():  digitalWrite(SONAR_POWER_PIN, LOW);
```

---

## 5. Beacon detection logic

`LoRaCL::receive()` internally calls `processBroadcastBundle()` for any bundle
with destination `ipn:0.0`.  This updates `LoRaCL::neighbors[]` with a fresh
`last_seen_ms = millis()`.

Detection in `main.cpp`:
```cpp
uint32_t listenStart = millis();
// ... receive loop ...
for (uint8_t i = 0; i < loRaCL.getNeighborCount(); i++) {
    if (nbrs[i].last_seen_ms >= listenStart) {
        beaconHeard = true; break;
    }
}
```

**Caveat**: `millis()` stops advancing during SAMD21 standby.  After waking,
`millis()` resumes where it left off (SysTick continues from before sleep on M0).
The `listenStart` capture happens after waking, so the comparison is always valid.

---

## 6. Bundle store behaviour (single-CLA node)

The sensor registers only one CLA (LoRa, index 0).
`BundleManager::removeForwardedBundles()` considers a bundle "fully forwarded" only
when BOTH LoRa AND LTE have sent it (`lte_required` is derived from
`received_via_cla != 1`).  For locally-generated sensor bundles,
`received_via_cla = 0xFF`, so `lte_required = true`, and bundles never get removed
by `removeForwardedBundles()`.

**Current workaround**: bundles expire after `BUNDLE_LIFETIME_MS = 24 h` via
`removeExpiredBundles()`.  The 32-slot FIFO also evicts oldest bundles when full.

**Permanent fix** (future work): add a method to BundleManager to mark a bundle
as "fully forwarded for single-CLA mode" after successful LoRa TX, then call
`removeForwardedBundles()`.  Or reduce `BUNDLE_LIFETIME_MS` to a few hours so
already-sent bundles cycle out faster.

---

## 7. Power budget estimate

| Phase | Duration | Current | Energy |
|-------|----------|---------|--------|
| GPS sync (time only) | 60 s | ~25 mA | 0.42 mAh |
| GPS sync (position, cold) | 5 min | ~25 mA | 2.1 mAh |
| Sonar measure (10 × 250 ms) | 2.5 s | ~10 mA | 0.007 mAh |
| LoRa listen window | 90 s | ~15 mA | 0.375 mAh |
| LoRa TX (1 bundle, ~100 ms) | 0.1 s | ~120 mA | 0.003 mAh |
| Deep standby | 540 s | ~0.05 mA | 0.0075 mAh |
| **Per cycle (10 min), no pos** | | | **~0.81 mAh** |
| **Per day (144 cycles)** | | | **~117 mAh** |

A 400 mAh LiPo gives roughly 3–4 days between charges (no solar).
Adding sonar power control (~26 mAh/day saved) extends to ~5 days.

---

## 8. GPS wiring (Adafruit Feather M0)

| Signal | Feather M0 pin | Notes |
|--------|----------------|-------|
| GPS TX → MCU RX | pin 0 (Serial1 RX) | UART from GPS |
| GPS RX ← MCU TX | pin 1 (Serial1 TX) | UART to GPS |
| GPS VCC enable | pin 11 | active-LOW MOSFET gate |
| GPS GND | GND | |
| GPS VCC | 3.3 V | through MOSFET switched by pin 11 |

The GPS is powered via pin 11 active-LOW (same pattern as the LoRaDTN example
sketch).  HIGH = GPS off, LOW = GPS on.

GPS baud sequence (same as base station):
1. Open Serial1 at 9600
2. Send `$PUBX,41,1,0007,0003,115200,0*18\r\n`
3. Re-open at 115200

---

## 9. LoRa radio (RFM95W / SX1276) on Feather M0

The Feather M0 LoRa has the SX1276 hard-wired:

| Signal | Feather M0 pin |
|--------|----------------|
| NSS (CS) | 8 |
| RESET | 4 |
| DIO0 | 3 |
| MOSI | 23 (SPI MOSI) |
| MISO | 22 (SPI MISO) |
| SCK | 24 (SPI SCK) |

These are already set in `config.h`.  The `LoRaCL` constructor takes them as
arguments and passes them to `LoRaSemDriver::init()`.

---

## 10. Extending the project

### Add BPEcho service
Already registered — the echo service (port 12161) is active.  To test:
```bash
bping -s ipn:<base_station_node>.12161 ipn:<sensor_node>.12161
```

### Add a second sensor (e.g. temperature)
1. Add fields to `SensorPayload` and bump `version` to 2.
2. Measure in `sensor.cpp`.
3. Update the gateway decoder accordingly.

### Multiple LoRa channels
Keep `LORA_SYNC_WORD` identical on sensor and base station; they must match for
packets to be heard.

### OTA update
Not implemented.  Field firmware updates require physical USB access or a custom
bootloader (e.g. Adafruit UF2).
