# CLAUDE.md — BPv7-M0UltrasonicDistanceSensor

This file provides guidance to Claude Code when working with this PlatformIO project.

## What this project is

A **low-power DTN sensor node** based on the Adafruit Feather M0 with RFM95 LoRa radio.
It measures distance using an **I2CXL-MaxSonar** ultrasonic sonar (I2C), pairs each reading
with GPS coordinates and timestamp, encodes everything as a BPv7 bundle, and transmits via
LoRa to the BPv7 base station (see `../BPv7-BaseStation`).

It sleeps in SAMD21 standby for ~8–9 minutes per cycle (wake at each 10-min slot boundary).

## Directory layout

```
src/
  config.h          — all hardware pins and timing constants (CHANGE THIS FIRST)
  main.cpp          — state machine (STARTUP → GPS_SYNC → MEASURE → LISTEN → SLEEP)
  rtc_manager.h/.cpp — RTCZero wrapper + GPS sync (adapted from BPv7-BaseStation)
  sensor.h/.cpp     — I2CXL-MaxSonar driver + SensorPayload builder

lib/ (referenced via lib_extra_dirs = ../BPv7-BaseStation/lib)
  TinyDTN/          — BPv7Codec, BundleManager, LoRaCL, BPEchoService
  lorasem_adapter/  — LoRaSemDriver (SX1276 wrapper)
  LoRaSem/          — LoRaSem (Sandeep Mistry LoRa wrapper)
```

## Build command

```bash
pio run
pio run --target upload
pio device monitor --baud 115200
```

The project shares the `TinyDTN` library with `BPv7-BaseStation` via `lib_extra_dirs`.
Both projects **must be siblings** inside the same PlatformIO `Projects/` folder.

## Key hardware differences from M4 base station

| Aspect                | M4 Base Station            | M0 Sensor (this project)    |
|-----------------------|----------------------------|-----------------------------|
| MCU                   | SAMD51 (M4)                | SAMD21 (M0)                 |
| RAM                   | 192 KB                     | 32 KB                       |
| RTC library           | custom SAMD51_RTC          | Arduino RTCZero             |
| Sleep API             | RTC->MODE2.CTRLA standby   | rtc_.standbyMode()          |
| GPS enable pin        | A2 (active LOW)            | pin 11 (active LOW)         |
| LoRa CS/RST/DIO0      | 2 / A1 / 3                 | **8 / 4 / 3**               |
| Display               | 128×64 OLED                | none                        |
| LTE modem             | SIM7080 on Serial2         | none                        |
| Battery sense         | A0 (custom divider)        | A7 (Feather built-in 2:1)   |
| ADC precision         | 12-bit                     | 10-bit (analogRead)         |

## State machine

```
STARTUP → GPS_SYNC → MEASURE → LISTEN (90 s) → [beacon?] TRANSMIT → SLEEP
                                               → [no beacon]        SLEEP
After SLEEP: wakes at next 10-min slot boundary, restarts at GPS_SYNC.
```

- **GPS_SYNC**: time-only sync each cycle (~60 s). Full position sync every
  `GPS_POS_INTERVAL_SLOTS` cycles (default 6 = 1 h).
- **MEASURE**: 10 sonar samples → outlier rejection → filtered mean + σ + count.
- **LISTEN**: LoRaCL in RX mode. Beacon detection via `last_seen_ms` in neighbor table.
  If base station appears as a new neighbor → beam heard → transmit.
- **TRANSMIT**: `bundleManager.forwardBundlesViaCLA("LoRa")` drains the store.
- **SLEEP**: `loRaCL.sleep()` → RTC alarm → `rtc_.standbyMode()`.

## Bundle payload

`SensorPayload` (21 bytes packed, defined in `sensor.h`):

| Field             | Type    | Notes                              |
|-------------------|---------|------------------------------------|
| version           | u8      | = 1                                |
| unixTimestamp     | u32     | seconds since 1970-01-01 UTC       |
| latE6 / lonE6     | i32×2   | degrees × 1 000 000                |
| altitudeM         | i16     | metres                             |
| distanceCm        | u16     | filtered mean (0xFFFF = invalid)   |
| distanceStdDevCm  | u8      | σ of valid samples                 |
| validSamples      | u8      | accepted samples out of 10         |
| batteryDV         | u8      | voltage × 10 (e.g. 37 = 3.7 V)    |
| temperatureC      | i8      | CPU temperature °C                 |
| satellites        | u8      | GPS satellites                     |
| flags             | u8      | b0=gpsValid b1=distValid b2=cached |

Bundle routing: `ipn:43120.1 → ipn:1.200`.
The base station stores and forwards to the gateway via LTE/MQTT.

## I2CXL-MaxSonar wiring

| Sensor pin | Feather M0 pin |
|------------|----------------|
| VCC        | 3.3 V          |
| GND        | GND            |
| SDA        | SDA (pin 20)   |
| SCL        | SCL (pin 21)   |

Add 4.7 kΩ pull-ups on SDA and SCL to 3.3 V if not already present.
Default I2C address: `0x70`.

## config.h — items to change before deployment

```cpp
#define NODE_NUMBER  43120UL   // unique per sensor node
#define GATEWAY_NODE 1UL       // ION/DTNex gateway node number
```

LoRa parameters **must match** the base station's `src/config.h`:
- `LORA_FREQ_MHZ`, `LORA_BW_KHZ`, `LORA_SF`, `LORA_CR`, `LORA_SYNC_WORD`

## Known limitations / TODOs

1. **Bundle removal after TX**: after LoRa forwarding, bundles stay in the store
   (marked `sent_via_LoRa`) until they expire (`BUNDLE_LIFETIME_MS = 24 h`) or are
   evicted by FIFO.  This is correct DTN behaviour but means 32 sent bundles can
   fill the store after ~5 h of no reception.  Fix: add `removeForwardedBundles()`
   override for single-CLA nodes in BundleManager.

2. **millis() after standby**: on SAMD21, `millis()` uses SysTick which stops in
   standby.  The elapsed sleep time is not added back to `millis()`.  Neighbor
   `last_seen_ms` comparison in the listen window is still correct because
   `listenStart` is captured *after* waking, but any code using `millis()` for
   long-term timing should use the RTC instead.

3. **GPS cold start**: first boot may take up to 10 min for satellite lock in a
   new location.  Subsequent time-only syncs typically complete in < 60 s.

4. **No LTE/MQTT**: the sensor relies entirely on encountering a base station beacon.
   If no base station is heard for 24 h, the 32-bundle store fills and oldest data
   is silently evicted (FIFO).

## Testing without hardware (DEBUG_NO_SLEEP=1)

Set in `platformio.ini`:
```ini
build_flags =
    -I src
    -DDEBUG_ENABLED=1
    -DDEBUG_NO_SLEEP=1
```
This replaces `rtc_.standbyMode()` with a timed busy-wait so the USB serial
link stays active.
