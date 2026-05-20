#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// Hardware: Adafruit Feather M0 with RFM95 LoRa (SAMD21)
// =============================================================================

// --- Indicator ---
#define LED_PIN          13     // Built-in green LED

// --- LoRa radio (SX1276 / RFM95W, hard-wired on Feather M0 LoRa) ---
#define LORA_SS_PIN       8
#define LORA_RST_PIN      4
#define LORA_DIO0_PIN     3

// --- GPS ---
// External GPS module on Serial1 (RX=pin0, TX=pin1).
// Enable pin is active-LOW: LOW = GPS powered, HIGH = GPS off.
#define GPS_ENABLE_PIN   11
#define GPS_SERIAL       Serial1

// --- I2CXL-MaxSonar (I2C on Wire, default address 0x70) ---
// The sensor is powered from 3.3V (always on).
// Optional: define a power-control pin here if you add a MOSFET.
#define SONAR_I2C_ADDR   0x70   // 7-bit I2C address

// --- Battery voltage (Feather M0 built-in divider on A7) ---
// Reading = analogRead(A7); vbat = reading * 2 * 3.3 / 1024
#define VBAT_PIN         A7

// =============================================================================
// LoRa radio parameters — MUST match the BPv7 base station
// =============================================================================
#define LORA_FREQ_MHZ    868.0f
#define LORA_BW_KHZ      250.0f
#define LORA_SF          9
#define LORA_CR          5       // 4/5 coding rate
#define LORA_TX_POWER    23      // dBm
#define LORA_SYNC_WORD   0x12   // Private network sync word
#define LORA_PREAMBLE    10

// =============================================================================
// BPv7 endpoint addressing
// =============================================================================
// This sensor's node number (unique per deployment — change before flashing)
#define NODE_NUMBER          268485086UL    // 0xA870

// Destination: ION/DTNex gateway node.  Bundles are forwarded there via LTE
// by the base station.  Service 200 = sonar sensor data (project-defined).
#define GATEWAY_NODE         268484800UL
#define GATEWAY_SERVICE      6UL

// Source service on this sensor node (arbitrary, 1 = sensor data)
#define SENSOR_SERVICE       1UL

// =============================================================================
// Timing
// =============================================================================
// Slot width — must match base station beacon cadence (1800 s = 30 min)
// Beacons transmitted at every :00 and :30 of each hour.
#define SLOT_INTERVAL_S      1800

// How many seconds before the slot boundary to wake up.
// GPS time-only sync ~60 s + sonar measurement ~25 s = ~85 s; 120 s leaves margin.
#define EARLY_WAKE_S         120

// On first boot: timeout before giving up on GPS and entering LoRa-time fallback
#define GPS_FALLBACK_TIMEOUT_MS  180000UL   // 3 min

// Maximum time to wait for GPS time fix (not position)
#define GPS_TIME_TIMEOUT_MS  300000UL   // 5 min

// Maximum time to wait for GPS 3D position fix
#define GPS_POS_TIMEOUT_MS   300000UL   // 5 min

// Re-acquire GPS position every N *slots* (12 slots × 30 min = 6 h)
#define GPS_POS_INTERVAL_SLOTS  12

// Timed listen window when RTC is synced: wake LoRa this many seconds before
// the slot boundary, then listen for LORA_BEACON_LISTEN_MS.
// Beacon arrives at the boundary; ±few-second jitter is covered by the window.
#define LORA_BEACON_EARLY_S    5UL      // wake LoRa this many seconds before slot
#define LORA_BEACON_LISTEN_MS  20000UL  // 20 s window (5 s early + 15 s late)

// Blind listen window used only when RTC has no time (no GPS, no beacon yet).
// In practice the no-GPS fallback (STATE_NO_GPS_FALLBACK) waits indefinitely;
// this constant covers the legacy 90-s path used if time is lost mid-operation.
#define LORA_LISTEN_WINDOW_MS  90000UL  // 90 s fallback blind window

// Bundle lifetime on sensor (bundles not yet forwarded expire after this)
#define BUNDLE_LIFETIME_MS    86400000UL // 24 hours

// =============================================================================
// Sensor measurement parameters
// =============================================================================
#define SONAR_NUM_SAMPLES    100     // measurements per cycle (~25 s total)
#define SONAR_SAMPLE_DELAY   150     // ms between samples
#define SONAR_MIN_CM         20      // reject readings < 20 cm (sensor min)
#define SONAR_MAX_CM         765     // reject readings > 765 cm (sensor max)
#define SONAR_OUTLIER_SIGMA  1.5f    // reject if > N sigma from mean

// =============================================================================
// Watchdog
// =============================================================================
#define WDT_TIMEOUT_MS       8000

// =============================================================================
// LoRaCL compile-time dependencies (required by LoRaCL.cpp)
// These must match the base station settings for the link to work.
// =============================================================================
// DTN_NODE_NUMBER is read by LoRaCL when building broadcast beacon source EID
#define DTN_NODE_NUMBER          NODE_NUMBER

// Listen-before-talk (LBT) / collision avoidance
#define LORA_COLLISION_AVOIDANCE_ENABLED  true
#define LORA_RSSI_THRESHOLD              -90     // dBm — channel "busy" threshold
#define LORA_INITIAL_BACKOFF_MIN_MS       10
#define LORA_INITIAL_BACKOFF_MAX_MS       50
#define LORA_MAX_RETRIES                  5
#define LORA_BACKOFF_MULTIPLIER           2

// Duty cycle limit (informational — not enforced on M0 in this build)
#define LORA_DUTY_CYCLE_LIMIT_PERCENT    10.0f

// Neighbour timeout (ms). Neighbours older than this are cleaned up.
#define NEIGHBOR_TIMEOUT_MS   (86400000UL)       // 24 h

// =============================================================================
// Debug macros
// =============================================================================
#if DEBUG_ENABLED
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

#endif // CONFIG_H
