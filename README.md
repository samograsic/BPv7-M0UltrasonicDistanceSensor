# BPv7 M0 Ultrasonic Distance Sensor

A low-power DTN (Delay-Tolerant Networking) sensor node that measures water /
snow / silo level with an ultrasonic sonar, pairs each reading with GPS time and
position, and forwards the result as a [BPv7 (RFC 9171)](https://datatracker.ietf.org/doc/rfc9171/)
bundle over LoRa to a base station.

Designed for unattended outdoor deployment on a small LiPo cell + solar panel:
the device sleeps in SAMD21 standby (~5 µA) between cycles and is on the air
for roughly 90 seconds out of every 30 minutes.

| | |
|---|---|
| MCU | Adafruit Feather M0 LoRa (SAMD21 + RFM95) |
| Radio | LoRa 434 MHz, BW 250 kHz, SF9, CR 4/5 |
| Sonar | MaxBotix I2CXL-MaxSonar (I2C, 20–765 cm) |
| GPS | Any NMEA module on Serial1, active-LOW power enable |
| Battery | 1S LiPo on Feather built-in connector, measured on A7 |
| Slot | 30 minutes, aligned to wall-clock :00 and :30 |
| Average draw | ~1 mA (typical) |

---

## Concept

```
   ┌─────────────────┐                       ┌──────────────────┐
   │   Sonar sensor  │  ─── LoRa bundle ───▶ │   Base station   │
   │   (this repo)   │   every 30 min        │  (BPv7-BaseStat.)│
   │                 │   at :00 and :30      │                  │
   │   ipn:NODE.1    │                       │   ipn:1.x        │
   └─────────────────┘                       └────────┬─────────┘
                                                      │ LTE/MQTT
                                                      ▼
                                              ┌──────────────────┐
                                              │  DTN gateway     │
                                              │  (DTNex / ION)   │
                                              └──────────────────┘
```

Every 30 minutes, aligned to wall-clock half-hour boundaries, the sensor:

1. Wakes from standby 120 seconds before the next slot boundary.
2. Powers up the GPS and acquires UTC time (~60 s typical), and a full 3-D
   position every 12 slots (6 hours).
3. Takes 100 ultrasonic samples, applies outlier rejection (1.5σ) and
   temperature compensation, producing a filtered mean and standard deviation.
4. Sleeps the radio, then wakes it 5 s before the slot boundary and listens for
   a 20-second window for the base-station beacon.
5. If a beacon is heard, forwards every queued bundle over LoRa.  If not, the
   bundles stay in the local store (up to 32, 24-hour lifetime) and are
   forwarded on a later cycle.
6. Returns to SAMD21 standby until the next wakeup.

If GPS time cannot be acquired on first boot (no sky view, etc.), the device
enters a fallback mode: keeps the GPS powered and the radio listening
indefinitely, and accepts an RTC sync from the BPv7 creation timestamp of any
bundle addressed to it.

---

## Modules used

### Required hardware

| Module | Where to source | Notes |
|---|---|---|
| Adafruit Feather M0 with RFM95 LoRa | [Adafruit #3178](https://www.adafruit.com/product/3178) | 433/868/915 MHz variants — match local regulations |
| MaxBotix I2CXL-MaxSonar-EZ (MB1202/MB1232/MB1242…) | MaxBotix or distributors | Any I2CXL variant works |
| GPS module with active-LOW enable | Adafruit Ultimate GPS, u-blox NEO-6/7/M8, etc. | NMEA at 9600 baud |
| 1S LiPo battery (≥ 400 mAh recommended) | — | Plugs into Feather JST |
| 2× 4.7 kΩ resistors | — | I2C pull-ups (if not on sonar PCB) |
| 433/868/915 MHz LoRa antenna | — | Match Feather variant |

### Software libraries

Pulled in automatically by PlatformIO from [`platformio.ini`](platformio.ini):

- [`RTCZero`](https://github.com/arduino-libraries/RTCZero) — SAMD21 RTC + standby
- [`Adafruit SleepyDog`](https://github.com/adafruit/Adafruit_SleepyDog) — WDT
- [`Adafruit GPS Library`](https://github.com/adafruit/Adafruit_GPS) — NMEA parsing

The DTN stack lives in [`lib/`](lib/) and is shared with the sibling
[`BPv7-BaseStation`](https://github.com/samograsic/BPv7-BaseStation) project:

- `TinyDTN/` — BPv7 codec, bundle manager, LoRa convergence layer, BPEcho service
- `LoRaSem/` + `lorasem_adapter/` — SX1276 driver and adapter

---

## How to connect

### Pin map (Feather M0 LoRa)

```
                       ┌──────────────────┐
                  RST ─┤                  ├─ Vin  ─── (USB / battery)
                  3V3 ─┤   Feather M0     ├─ GND
                 AREF ─┤   LoRa RFM95     ├─ USB
                  GND ─┤                  ├─ D13 ─── LED (built-in)
                   A0 ─┤                  ├─ D12
                   A1 ─┤                  ├─ D11 ─── GPS_ENABLE (active LOW)
                   A2 ─┤                  ├─ D10
                   A3 ─┤                  ├─ D9
                   A4 ─┤                  ├─ D6
                   A5 ─┤                  ├─ D5
                  SCK ─┤                  ├─ SCL ─── Sonar SCL
                 MOSI ─┤                  ├─ SDA ─── Sonar SDA
                 MISO ─┤                  ├─ D1  ─── GPS RX  (Serial1 TX)
                  RX0 ─┤    GPS TX in     ├─ D0  ─── GPS TX  (Serial1 RX)
                       └──────────────────┘
                              D3 ─── (LoRa DIO0, hard-wired)
                              D4 ─── (LoRa RST,  hard-wired)
                              D8 ─── (LoRa CS,   hard-wired)
                              A7 ─── (battery /2, hard-wired)
```

### I2CXL-MaxSonar wiring

| Sonar pin | Feather M0 | Notes |
|---|---|---|
| GND | GND | — |
| V+ | 3.3 V | sensor accepts 2.7–5.5 V |
| SDA | SDA | add 4.7 kΩ pull-up to 3.3 V if not on sensor PCB |
| SCL | SCL | add 4.7 kΩ pull-up to 3.3 V if not on sensor PCB |
| TX  | leave open | optional UART output, unused here |
| AN  | leave open | analog output, unused here |
| RX / Mode | leave open | trigger pin, unused (I2C-triggered) |

Default I2C address is `0x70` (configurable on the sensor; matches
`SONAR_I2C_ADDR` in [`src/config.h`](src/config.h)).

### GPS wiring

The GPS is powered through a small MOSFET / load switch whose gate is tied to
`GPS_ENABLE_PIN = D11` (active-LOW, so LOW = powered, HIGH = off).  This lets
the MCU cut the ~25 mA GPS draw during standby.

| GPS pin | Feather M0 | Notes |
|---|---|---|
| VIN | 3.3 V through MOSFET, gate on D11 | active-LOW enable |
| GND | GND | — |
| TX  | D0 (Serial1 RX) | NMEA out at 9600 baud |
| RX  | D1 (Serial1 TX) | unused by software (no config commands sent) |

If your GPS module has its own enable / standby pin and you do not use a
MOSFET, connect that pin to D11 instead and adjust the polarity in
`rtc_manager.cpp` if needed.

### LoRa antenna

The Feather M0 LoRa has no antenna mounted.  Solder a 1/4-wave wire (about
**8.2 cm** for 868 MHz, **17.3 cm** for 434 MHz) to the `ANT` pad, or fit a
u.FL connector and use a proper antenna.  **Never power the radio without an
antenna attached** — you can destroy the SX1276 output stage.

---

## Build and flash

This is a [PlatformIO](https://platformio.org/) project.  Open the folder in
VSCode with the PlatformIO extension, or use the CLI:

```bash
pio run                 # compile
pio run --target upload # flash via USB (Feather appears as a serial device)
pio device monitor      # 115200 baud — watches the state machine
```

The build flags in [`platformio.ini`](platformio.ini) control runtime behaviour:

| Flag | Default | Meaning |
|---|---|---|
| `DEBUG_ENABLED` | `1` | Enables serial output and the hardware self-test |
| `DEBUG_NO_SLEEP` | `1` | Replaces SAMD21 standby with a `delay()` so USB stays alive |

For a real deployment, set both to `0` for minimum current draw and the longest
USB-disconnected operation.

---

## Configuration

Before flashing a unit for deployment, edit [`src/config.h`](src/config.h):

```c
#define NODE_NUMBER       43120UL    // unique per sensor (must not collide)
#define GATEWAY_NODE      268484800UL // ipn: node of the DTN gateway
#define GATEWAY_SERVICE   6UL        // ipn: service number the gateway listens on

#define LORA_FREQ_MHZ     434.0f     // must match the base station
#define LORA_SF           9
#define LORA_BW_KHZ       250.0f
#define LORA_SYNC_WORD    0x12
```

**All LoRa radio parameters must match the base station** for the link to
work.  See the [`BPv7-BaseStation`](https://github.com/samograsic/BPv7-BaseStation)
config for the canonical values.

---

## Bundle payload

The sensor encodes each measurement as a CBOR map carried inside a BPv7
bundle (`source ipn:NODE_NUMBER.1`, `destination ipn:GATEWAY_NODE.GATEWAY_SERVICE`).

The CBOR map uses the same numeric-key schema as the base station's status
bundle, so the gateway can use one decoder for both node types.

| key | field | type | notes |
|---:|---|---|---|
|  1 | `station_id` | uint | `NODE_NUMBER` |
|  2 | `timestamp_ms` | uint | ms since DTN epoch (2000-01-01 UTC), `0` if unknown |
|  4 | `cpu_temperature` | float | °C — also used for speed-of-sound correction |
|  5 | `battery_voltage` | float | V |
| 13 | `gps_latitude`  | float / null | degrees |
| 14 | `gps_longitude` | float / null | degrees |
| 15 | `gps_altitude`  | float / null | metres |
| 22 | `sonar_distance_cm` | float / null | temperature-compensated mean |
| 23 | `sonar_stddev_cm`   | float / null | filtered standard deviation |
| 25 | `sonar_delta_cm`    | float / null | signed delta vs previous reading |

GPS keys encode CBOR `null` (`0xF6`) when no position fix has ever been
obtained.  Sonar keys encode `null` when fewer than 3 of 100 samples passed the
range and outlier checks.

---

## Operation cycle

```
   wake (-120 s)        slot boundary             next wake (-120 s)
        │                    │                          │
   ┌────▼───┐ ┌────┐ ┌──────┬▼─────┐ ┌──────┐ ┌────────▼───────┐
   │GPS sync│ │meas│ │wait  │listen│ │ TX?  │ │  SAMD21 standby │
   │ ~60 s  │ │~25s│ │ off  │ 20 s │ │ ~5 s │ │   ~28 minutes   │
   └────────┘ └────┘ └──────┴──────┘ └──────┘ └─────────────────┘
   ◀── awake ────────────────────────────────▶ ◀───── asleep ───▶
```

The full state machine is in [`src/main.cpp`](src/main.cpp):

| State | Purpose |
|---|---|
| `STARTUP` | One-shot init, only on cold boot |
| `GPS_SYNC` | Time fix every slot; full position fix every 12 slots (6 h) |
| `MEASURE` | 100 sonar samples → outlier rejection → CBOR bundle |
| `LISTEN` | 5 s pre-slot + 20 s receive window — wait for the beacon |
| `TRANSMIT` | If beacon heard: drain bundle store via LoRa |
| `SLEEP` | I²C off, radio sleep, WDT off, RTC alarm, standby |
| `NO_GPS_FALLBACK` | First-boot bootstrap when no GPS fix is available |

---

## Project layout

```
src/
  config.h           hardware pins, timing constants, LoRa params
  main.cpp           state machine
  rtc_manager.h/.cpp RTCZero + Adafruit GPS + UTC ↔ DTN epoch + alarm
  sensor.h/.cpp      I2CXL-MaxSonar driver, statistics, CBOR encoder

lib/
  TinyDTN/           BPv7 codec, BundleManager, LoRaCL, BPEcho
  LoRaSem/           SX1276 driver
  lorasem_adapter/   LoRaSemDriver wrapper
  cpu_utils/         (header-only helpers)
```

---

## Related projects

- [`BPv7-BaseStation`](https://github.com/samograsic/BPv7-BaseStation) — the
  M4 base station that beacons every 30 min, listens for sensor uplinks, and
  forwards them to a DTN gateway over LTE.

---

## License

Add a license of your choice (MIT, Apache-2.0, GPL-3.0…) before publishing.
