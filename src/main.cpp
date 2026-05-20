/**
 * BPv7-M0UltrasonicDistanceSensor
 *
 * Adafruit Feather M0 with RFM95 LoRa + I2CXL-MaxSonar ultrasonic sensor + GPS
 *
 * Cycle (every 10 minutes, aligned to :00/:10/:20/... slot boundaries):
 *   GPS_SYNC  → acquire time (and position periodically)
 *   MEASURE   → 10 sonar samples → statistical filtering
 *   LISTEN    → 90 s LoRa receive window — wait for base station beacon
 *   TRANSMIT  → if beacon heard: forward all stored bundles via LoRa
 *   SLEEP     → RTC alarm → deep standby
 *
 * Bundle flow:
 *   Sensor (ipn:NODE.1) ─LoRa→ Base Station ─LTE→ DTNex Gateway (ipn:1.200)
 */

#include <Arduino.h>
#include <Adafruit_SleepyDog.h>
#include <Wire.h>

#include "config.h"
#include "rtc_manager.h"
#include "sensor.h"

// TinyDTN (from base station via lib_extra_dirs)
#include "Bundle.h"
#include "BPv7Codec.h"
#include "BundleManager.h"
#include "LoRaCL.h"
#include "BPEchoService.h"

// =============================================================================
// Global objects
// =============================================================================

LoRaCL loRaCL(
    LORA_SS_PIN,  LORA_DIO0_PIN, LORA_RST_PIN,
    LORA_FREQ_MHZ, LORA_BW_KHZ,
    LORA_SF, LORA_CR, LORA_TX_POWER,
    LORA_SYNC_WORD, LORA_PREAMBLE);

BundleManager  bundleManager;
BPEchoService  echoService(&bundleManager);

// =============================================================================
// State machine
// =============================================================================
enum SensorState : uint8_t {
    STATE_STARTUP          = 0,
    STATE_GPS_SYNC         = 1,
    STATE_MEASURE          = 2,
    STATE_LISTEN           = 3,
    STATE_TRANSMIT         = 4,
    STATE_SLEEP            = 5,
    STATE_NO_GPS_FALLBACK  = 6,  // GPS timed out — sync time from LoRa beacon
};

static SensorState state = STATE_STARTUP;

// =============================================================================
// RTC / sleep wiring
// =============================================================================
volatile bool alarmFired = false;

void onAlarm() {
    alarmFired = true;
}

// =============================================================================
// DTN time callback for BundleManager
// =============================================================================
static uint64_t getDtnTimeMs() {
    return rtcManager.getDtnTimeMs();
}

// =============================================================================
// GPS position state (persists across sleep cycles)
// =============================================================================
static float  cachedLat  = 0;
static float  cachedLon  = 0;
static float  cachedAlt  = 0;
static uint8_t cachedSats = 0;
static bool   posEverValid = false;
static uint32_t slotsSincePosFix = 0;   // counts cycles since last full GPS fix
static bool     firstBoot = true;           // cleared after the first successful GPS time lock
static uint16_t lastValidDistanceCm = 0xFFFF; // 0xFFFF = no previous valid reading

// =============================================================================
// Hardware self-test — called once at boot before the watchdog is armed.
// Probes every peripheral and prints a clear PASS/FAIL line for each.
// GPS is tested at both 9600 (raw boot baud) and 115200 (after baud-switch)
// so we can distinguish a power/wiring fault from a baud-switch failure.
// =============================================================================
#if DEBUG_ENABLED
static void runHardwareSelfTest() {
    DEBUG_PRINTLN("\n=== Hardware Self-Test ===");

    // --- Battery & CPU temperature (always readable) ---
    float vbat = sensorManager.readBatteryVoltage();
    float temp = sensorManager.readInternalTemperature();
    DEBUG_PRINTF("  Battery:     %.2f V  %s\n", vbat, vbat > 3.2f ? "OK" : "LOW");
    DEBUG_PRINTF("  Temperature: %.1f C\n", temp);

    // --- LoRa (already initialised — fatal trap caught any failure earlier) ---
    DEBUG_PRINTF("  LoRa SX1276: OK  (%.1f MHz  SF%d  BW%.0f kHz  sync=0x%02X)\n",
                 LORA_FREQ_MHZ, LORA_SF, LORA_BW_KHZ, LORA_SYNC_WORD);

    // --- Sonar: I2C presence check ---
    Wire.beginTransmission(SONAR_I2C_ADDR);
    uint8_t i2cErr = Wire.endTransmission();
    if (i2cErr == 0) {
        DEBUG_PRINTF("  Sonar I2C:   OK  (0x%02X)\n", SONAR_I2C_ADDR);
    } else {
        DEBUG_PRINTF("  Sonar I2C:   FAIL  err=%d — check SDA/SCL and 4.7k pull-ups\n", i2cErr);
    }

    // --- GPS: power on and probe at 9600 baud (factory default) ---
    DEBUG_PRINTF("  GPS enable:  pin %d LOW\n", GPS_ENABLE_PIN);
    pinMode(GPS_ENABLE_PIN, OUTPUT);
    digitalWrite(GPS_ENABLE_PIN, LOW);
    delay(1000);

    GPS_SERIAL.begin(9600);
    delay(500);

    {
        uint32_t t0 = millis();
        uint32_t rxCount = 0;
        char     buf[82];
        uint8_t  bLen   = 0;
        bool     gotLine = false;

        while (millis() - t0 < 2000) {
            while (GPS_SERIAL.available()) {
                char c = (char)GPS_SERIAL.read();
                rxCount++;
                if (!gotLine && bLen < (uint8_t)(sizeof(buf) - 1)) {
                    buf[bLen++] = c;
                    if (c == '\n') gotLine = true;
                }
            }
        }
        buf[bLen] = '\0';
        // strip trailing CR/LF for clean print
        while (bLen > 0 && (buf[bLen-1] == '\r' || buf[bLen-1] == '\n'))
            buf[--bLen] = '\0';

        if (rxCount == 0) {
            DEBUG_PRINTF("  GPS @9600:   FAIL — 0 bytes received"
                         "  (check enable pin %d and Serial1 RX/TX wiring)\n",
                         GPS_ENABLE_PIN);
        } else {
            DEBUG_PRINTF("  GPS @9600:   OK   rx=%lu B   %s\n",
                         rxCount, gotLine ? buf : "(partial — no newline yet)");
        }
    }

    GPS_SERIAL.end();
    digitalWrite(GPS_ENABLE_PIN, HIGH);   // power off — openGPSSerial() will re-enable

    DEBUG_PRINTLN("=== Self-Test Done ===\n");
}
#endif // DEBUG_ENABLED

// =============================================================================
// setup()
// =============================================================================
void setup() {
#if DEBUG_ENABLED
    // Wait for USB serial (max 5 s, so we don't stall in field)
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 5000)) {}
    DEBUG_PRINTLN("=== BPv7 M0 Ultrasonic Sensor ===");
    DEBUG_PRINTF("Node: ipn:%lu.%lu  Gateway: ipn:%lu.%lu\n",
                 NODE_NUMBER, SENSOR_SERVICE, GATEWAY_NODE, GATEWAY_SERVICE);
#endif

    // Blink LED to signal boot
    pinMode(LED_PIN, OUTPUT);
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
    }

    // Ensure GPS is off until we need it
    pinMode(GPS_ENABLE_PIN, OUTPUT);
    digitalWrite(GPS_ENABLE_PIN, HIGH);

    // I2C for sonar
    Wire.begin();

    // Hardware RTC
    rtcManager.initialize();
    rtcManager.attachAlarmCallback(onAlarm);

    // LoRa radio
    if (!loRaCL.initialize()) {
        DEBUG_PRINTLN("FATAL: LoRa init failed");
        while (true) { Watchdog.reset(); delay(1000); }
    }
    loRaCL.sleep();   // keep radio in sleep until STATE_LISTEN needs it
    DEBUG_PRINTLN("LoRa: OK (sleeping)");

    // Bundle manager
    bundleManager.setLocalEID(NODE_NUMBER, 0);
    bundleManager.setTimeSource(getDtnTimeMs);
    bundleManager.registerCLA(&loRaCL);
    bundleManager.registerService(&echoService);

    // Sonar
    sensorManager.initialize();

    // Hardware self-test (before watchdog so GPS probing can take its time)
#if DEBUG_ENABLED
    runHardwareSelfTest();
#endif

    // Enable watchdog
    Watchdog.enable(WDT_TIMEOUT_MS);

    // First thing: GPS sync (time + position)
    state = STATE_GPS_SYNC;
    DEBUG_PRINTLN("Setup complete — entering state machine");
}

// =============================================================================
// Bundle destination filter
//
// Decodes just enough of a raw CBOR bundle to read the primary-block destination
// EID.  Returns true only when destination.node_number matches NODE_NUMBER.
// Bundles addressed to other nodes (transit, overheard traffic) are rejected
// before they reach BundleManager — this sensor never relays traffic.
// =============================================================================
static bool isForThisNode(const uint8_t* data, size_t len) {
    Bundle tmp;
    if (!BPv7Codec::decode(data, len, tmp)) return false;
    return tmp.primary.destination.node_number == NODE_NUMBER;
}

// =============================================================================
// loop()
// =============================================================================
void loop() {
    Watchdog.reset();

    switch (state) {

    // -------------------------------------------------------------------------
    case STATE_STARTUP:
        // Only reached on hard reset after initial GPS fix; go back to GPS sync
        state = STATE_GPS_SYNC;
        break;

    // -------------------------------------------------------------------------
    case STATE_GPS_SYNC: {
        // Decide whether this cycle needs a full position fix
        bool needPos = !posEverValid || (slotsSincePosFix >= GPS_POS_INTERVAL_SLOTS);

        DEBUG_PRINTF("GPS sync: needPos=%s (slots since fix: %lu)\n",
                     needPos ? "yes" : "no", (unsigned long)slotsSincePosFix);

        // On first boot give GPS 3 min; if still no lock → LoRa-time fallback.
        // On subsequent cycles use the normal per-mode timeouts.
        uint32_t timeout = firstBoot ? GPS_FALLBACK_TIMEOUT_MS
                         : (needPos ? GPS_POS_TIMEOUT_MS : GPS_TIME_TIMEOUT_MS);
        bool ok = rtcManager.syncWithGPS(timeout, needPos);

        if (ok) firstBoot = false;

        if (!ok && !rtcManager.isTimeSynced()) {
            if (firstBoot) {
                // First boot, no GPS fix — try to get time from a LoRa beacon
                DEBUG_PRINTLN("GPS: no time in 3 min — entering LoRa-time fallback");
                state = STATE_NO_GPS_FALLBACK;
            } else {
                // Mid-operation loss — retry after a short pause
                DEBUG_PRINTLN("GPS: no time fix, retrying this cycle in 60 s");
                delay(60000);
                Watchdog.reset();
                state = STATE_GPS_SYNC;
            }
            break;
        }

        // Update position cache if we got a fresh fix
        if (needPos && rtcManager.isPositionValid()) {
            cachedLat   = rtcManager.getLatitude();
            cachedLon   = rtcManager.getLongitude();
            cachedAlt   = rtcManager.getAltitude();
            cachedSats  = rtcManager.getGpsSatellites();
            posEverValid = true;
            slotsSincePosFix = 0;
        } else {
            slotsSincePosFix++;
        }

        state = STATE_MEASURE;
        break;
    }

    // -------------------------------------------------------------------------
    case STATE_MEASURE: {
        DEBUG_PRINTLN("--- Sensor measurement ---");

        Wire.begin();  // re-enable I2C (was shut down in STATE_SLEEP)
        SonarResult sonar = sensorManager.measure();

        // Delta vs last valid reading (persists across sleep cycles via static)
        int16_t deltaCm   = (int16_t)INT16_MIN;
        bool    deltaValid = false;
        if (sonar.valid && lastValidDistanceCm != 0xFFFF) {
            deltaCm   = (int16_t)sonar.meanCm - (int16_t)lastValidDistanceCm;
            deltaValid = true;
            DEBUG_PRINTF("Distance delta: %+d cm\n", deltaCm);
        }
        if (sonar.valid) lastValidDistanceCm = sonar.meanCm;

        // Build CBOR payload and create bundle
        bool gpsValid  = rtcManager.isPositionValid();
        bool gpsCached = posEverValid && !gpsValid;

        uint8_t cborBuf[128];
        size_t  cborLen = 0;
        bool encoded = sensorManager.buildCBORPayload(
            cborBuf, sizeof(cborBuf), &cborLen,
            rtcManager.getUnixEpoch(),
            cachedLat, cachedLon, cachedAlt, cachedSats,
            gpsValid, gpsCached, sonar,
            deltaCm, deltaValid);

        EndpointID dest(GATEWAY_NODE, GATEWAY_SERVICE);
        if (encoded) {
            bool stored = bundleManager.createBundle(
                dest, cborBuf, cborLen,
                BUNDLE_LIFETIME_MS,
                SENSOR_SERVICE);
            if (stored) {
                DEBUG_PRINTF("Bundle queued: %d B CBOR\n", (int)cborLen);
            } else {
                DEBUG_PRINTLN("Bundle store full — oldest will be evicted by FIFO");
            }
        } else {
            DEBUG_PRINTLN("CBOR encode failed (buffer too small?)");
        }

        // Remove bundles older than BUNDLE_LIFETIME_MS
        bundleManager.removeExpiredBundles();

        state = STATE_LISTEN;
        break;
    }

    // -------------------------------------------------------------------------
    // STATE_LISTEN
    //
    // A) RTC SYNCED — power-saving timed listen:
    //    Base station beacons at every 30-min slot boundary (0:00, 0:30, 1:00 …).
    //    LoRa sleeps until LORA_BEACON_EARLY_S before the boundary, then
    //    startReceive() opens a LORA_BEACON_LISTEN_MS (~20 s) window.
    //
    // B) RTC NOT SYNCED — should not normally reach here (the no-GPS fallback
    //    handles this), but fall back to a 90-s blind window just in case.
    // -------------------------------------------------------------------------
    case STATE_LISTEN: {
        static uint8_t rxBuf[255];
        bool beaconHeard = false;

        if (rtcManager.isTimeSynced()) {
            // --- A: timed listen ---
            uint32_t nowSec   = (uint32_t)rtcManager.getHours()   * 3600UL
                              + (uint32_t)rtcManager.getMinutes() * 60UL
                              + (uint32_t)rtcManager.getSeconds();
            uint32_t nextSlot  = ((nowSec / SLOT_INTERVAL_S) + 1UL) * SLOT_INTERVAL_S;
            int32_t  secsUntil = (int32_t)(nextSlot - nowSec);  // 1 … SLOT_INTERVAL_S

            DEBUG_PRINTF("LoRa timed listen: slot in %lds"
                         "  (radio on at -%lus, window %lums)\n",
                         (long)secsUntil,
                         (unsigned long)LORA_BEACON_EARLY_S,
                         (unsigned long)LORA_BEACON_LISTEN_MS);

            // Overrun check: if we woke EARLY_WAKE_S before the slot and GPS +
            // measurement consumed more than that budget, secsUntil wraps to
            // nearly a full slot.  Anything > EARLY_WAKE_S means we missed
            // this slot — go straight to sleep for the next cycle.
            int32_t waitSec = secsUntil - (int32_t)LORA_BEACON_EARLY_S;
            if (waitSec < 0 || waitSec > (int32_t)EARLY_WAKE_S) {
                DEBUG_PRINTF("LoRa: beacon window missed (secsUntil=%lds) — sleeping\n",
                             (long)secsUntil);
                state = STATE_SLEEP;
                break;
            }
            if (waitSec > 0) {
                DEBUG_PRINTF("LoRa: radio off, waiting %lds\n", (long)waitSec);
                uint32_t waitEnd = millis() + (uint32_t)waitSec * 1000UL;
                while ((int32_t)(waitEnd - millis()) > 0) {
                    Watchdog.reset();
                    delay(500);
                }
            }

            // Wake radio into continuous RX
            loRaCL.startReceive();
            uint32_t listenStart = millis();
            DEBUG_PRINTF("LoRa: RX on, listening %lums\n",
                         (unsigned long)LORA_BEACON_LISTEN_MS);

            while ((millis() - listenStart) < LORA_BEACON_LISTEN_MS) {
                Watchdog.reset();

                if (loRaCL.available()) {
                    size_t rxLen = 0;
                    if (loRaCL.receive(rxBuf, sizeof(rxBuf), &rxLen) && rxLen > 0) {
                        if (isForThisNode(rxBuf, rxLen)) {
                            bundleManager.processReceivedBundle(
                                rxBuf, rxLen,
                                loRaCL.getLastRSSI(), loRaCL.getLastSNR(), "LoRa");
                        } else {
                            DEBUG_PRINTLN("RX: bundle not for us — discarded");
                        }
                    }
                }

                const LoRaNeighbor* nbrs = loRaCL.getNeighbors();
                for (uint8_t i = 0; i < loRaCL.getNeighborCount(); i++) {
                    if (nbrs[i].last_seen_ms >= listenStart) {
                        DEBUG_PRINTF("Beacon from node %lu  RSSI=%d  SNR=%.1f\n",
                                     nbrs[i].node_number, nbrs[i].rssi, nbrs[i].snr);
                        beaconHeard = true;
                        break;
                    }
                }
                if (beaconHeard) break;
                delay(10);
            }

            DEBUG_PRINTF("Listen done: beacon=%s  (%lums)\n",
                         beaconHeard ? "YES" : "no",
                         (unsigned long)(millis() - listenStart));

        } else {
            // --- B: no time sync — 90-s blind window ---
            DEBUG_PRINTF("LoRa blind listen: %lus (no time sync)\n",
                         LORA_LISTEN_WINDOW_MS / 1000);
            loRaCL.startReceive();
            uint32_t listenStart = millis();

            while ((millis() - listenStart) < LORA_LISTEN_WINDOW_MS) {
                Watchdog.reset();

                if (loRaCL.available()) {
                    size_t rxLen = 0;
                    if (loRaCL.receive(rxBuf, sizeof(rxBuf), &rxLen) && rxLen > 0) {
                        if (isForThisNode(rxBuf, rxLen)) {
                            bundleManager.processReceivedBundle(
                                rxBuf, rxLen,
                                loRaCL.getLastRSSI(), loRaCL.getLastSNR(), "LoRa");
                        } else {
                            DEBUG_PRINTLN("RX: bundle not for us — discarded");
                        }
                    }
                }

                const LoRaNeighbor* nbrs = loRaCL.getNeighbors();
                for (uint8_t i = 0; i < loRaCL.getNeighborCount(); i++) {
                    if (nbrs[i].last_seen_ms >= listenStart) {
                        beaconHeard = true;
                        break;
                    }
                }
                if (beaconHeard) break;
                delay(10);
            }
            DEBUG_PRINTF("Listen done: beacon=%s\n", beaconHeard ? "YES" : "no");
        }

        state = beaconHeard ? STATE_TRANSMIT : STATE_SLEEP;
        break;
    }

    // -------------------------------------------------------------------------
    case STATE_TRANSMIT: {
        DEBUG_PRINTLN("Transmitting stored bundles via LoRa...");

        uint16_t sent = bundleManager.forwardBundlesViaCLA("LoRa");
        DEBUG_PRINTF("TX: %d bundle(s) forwarded\n", sent);

        // Brief additional listen to receive echo/ACKs from base station
        uint32_t t0 = millis();
        static uint8_t rxBuf[255];
        while (millis() - t0 < 5000) {
            Watchdog.reset();
            if (loRaCL.available()) {
                size_t rxLen = 0;
                if (loRaCL.receive(rxBuf, sizeof(rxBuf), &rxLen) && rxLen > 0) {
                    if (isForThisNode(rxBuf, rxLen)) {
                        bundleManager.processReceivedBundle(
                            rxBuf, rxLen,
                            loRaCL.getLastRSSI(), loRaCL.getLastSNR(), "LoRa");
                    } else {
                        DEBUG_PRINTLN("RX: bundle not for us — discarded");
                    }
                }
            }
            delay(10);
        }

        state = STATE_SLEEP;
        break;
    }

    // -------------------------------------------------------------------------
    // STATE_NO_GPS_FALLBACK
    //
    // Entered on first boot when GPS fails to deliver a time fix within 3 min.
    // Strategy:
    //   1. Keep GPS on and poll it in the background.
    //   2. Measure distance and queue a bundle (coords = 0, timestamp = 0).
    //   3. Listen for any LoRa bundle from the base station; decode its BPv7
    //      creation timestamp and use that to set the RTC.
    //   4. Once time is known (from GPS or LoRa), close GPS and go to MEASURE
    //      so a properly-timestamped bundle is also queued.
    // -------------------------------------------------------------------------
    case STATE_NO_GPS_FALLBACK: {
        DEBUG_PRINTLN("--- No-GPS fallback ---");
        DEBUG_PRINTLN("    GPS on (background) + LoRa RX listening for base-station beacon.");
        DEBUG_PRINTLN("    Will wait indefinitely — beacon may take up to ~1 h.");

        // Keep GPS powered (syncWithGPS closed it; reopen at 9600)
        rtcManager.openGPSSerial();

        // --- Measure distance now (timestamp unknown → 0, coords → 0) ---
        SonarResult fbSonar = sensorManager.measure();

        {
            uint8_t fbCbor[128];
            size_t  fbLen = 0;
            bool fbEncoded = sensorManager.buildCBORPayload(
                fbCbor, sizeof(fbCbor), &fbLen,
                0,                     // unixTimestamp = 0 (unknown)
                0.0f, 0.0f, 0.0f, 0,  // lat/lon/alt/sats all zero
                false, false,          // gpsValid=false, gpsCached=false
                fbSonar,
                0, false);             // no delta on first measurement

            if (fbEncoded) {
                EndpointID dest(GATEWAY_NODE, GATEWAY_SERVICE);
                bool stored = bundleManager.createBundle(
                    dest, fbCbor, fbLen, BUNDLE_LIFETIME_MS, SENSOR_SERVICE);
                DEBUG_PRINTF("Fallback bundle %s (%d B CBOR, no timestamp)\n",
                             stored ? "queued" : "DROPPED (store full)", (int)fbLen);
            }
        }

        // --- Listen until we get time from LoRa or GPS ---
        static uint8_t fbRxBuf[255];

        // Ensure radio is in continuous RX mode before the listen loop.
        // initialize() leaves it in RX, but be explicit in case we re-enter.
        loRaCL.startReceive();

        uint32_t listenStart  = millis();
        uint32_t lastReport   = 0;
        bool     gotTime      = false;

        // Minimum valid DTN creation timestamp: 2023-01-01 = 725846400 s from
        // DTN epoch (2000-01-01).  Reject anything older as clock-not-set garbage.
        const uint64_t kMinValidDtnMs = 725846400000ULL;

        while (!gotTime) {
            Watchdog.reset();

            // Poll GPS in background
            if (rtcManager.pollGPSTime()) {
                DEBUG_PRINTLN("Fallback: time acquired from GPS");
                gotTime = true;
                break;
            }

            // Check LoRa for any bundle carrying a usable creation timestamp.
            // Only bundles addressed to this node are accepted; all others are
            // discarded without touching BundleManager.
            if (loRaCL.available()) {
                size_t rxLen = 0;
                if (loRaCL.receive(fbRxBuf, sizeof(fbRxBuf), &rxLen) && rxLen > 0) {

                    Bundle rxBundle;
                    if (BPv7Codec::decode(fbRxBuf, rxLen, rxBundle)) {
                        if (rxBundle.primary.destination.node_number != NODE_NUMBER) {
                            DEBUG_PRINTLN("RX: bundle not for us — discarded");
                        } else {
                            uint64_t dtnMs = rxBundle.primary.creation_timestamp;
                            if (dtnMs >= kMinValidDtnMs) {
                                // DTN epoch offset: 2000-01-01 → 1970-01-01 = 946684800 s
                                uint32_t unixSec = (uint32_t)(dtnMs / 1000ULL) + 946684800UL;
                                rtcManager.setRTCFromUnixEpoch(unixSec);
                                DEBUG_PRINTF("Fallback: RTC synced from LoRa bundle"
                                             "  dtn=%llu ms  unix=%lu\n",
                                             (unsigned long long)dtnMs, unixSec);
                                gotTime = true;
                            } else {
                                DEBUG_PRINTF("Fallback: bundle timestamp too old"
                                             " (%llu ms) — ignoring\n",
                                             (unsigned long long)dtnMs);
                            }

                            bundleManager.processReceivedBundle(
                                fbRxBuf, rxLen,
                                loRaCL.getLastRSSI(), loRaCL.getLastSNR(), "LoRa");
                        }
                    }
                }
            }

            // Progress heartbeat every 60 s — show HH:MM so long waits are readable
            uint32_t elapsed = millis() - listenStart;
            if (elapsed - lastReport >= 60000) {
                lastReport = elapsed;
                uint32_t elapsedSec = elapsed / 1000;
                DEBUG_PRINTF("Fallback: waiting for beacon... %02lu:%02lu elapsed"
                             "  (GPS rx active, LoRa listening)\n",
                             elapsedSec / 3600, (elapsedSec % 3600) / 60);
            }

            delay(10);
        }

        rtcManager.closeGPSSerial();
        firstBoot = false;

        // Proceed to measure a fresh bundle now that the clock is set
        state = STATE_MEASURE;
        break;
    }

    // -------------------------------------------------------------------------
    case STATE_SLEEP: {
        DEBUG_PRINTLN("--- Preparing for deep sleep ---");

        // Shut down peripherals before standby
        Wire.end();    // disable I2C — re-enabled in STATE_MEASURE
        loRaCL.sleep();

        // Compute how long the device will actually sleep (= time until the alarm
        // fires, i.e. next slot boundary minus EARLY_WAKE_S).  Capture this before
        // setAlarmForNextSlot() so both real and debug paths use the same value.
        {
            uint32_t nowSec   = (uint32_t)rtcManager.getHours()   * 3600UL
                              + (uint32_t)rtcManager.getMinutes() * 60UL
                              + (uint32_t)rtcManager.getSeconds();
            uint32_t nextSlot = ((nowSec / SLOT_INTERVAL_S) + 1UL) * SLOT_INTERVAL_S;
            int32_t  sleepSec = (int32_t)(nextSlot - nowSec) - (int32_t)EARLY_WAKE_S;
            if (sleepSec < 0) sleepSec = 0;

            // Set alarm for next slot (wake EARLY_WAKE_S before the boundary)
            alarmFired = false;
            rtcManager.setAlarmForNextSlot(EARLY_WAKE_S);

            // Turn off LED
            digitalWrite(LED_PIN, LOW);

            Watchdog.disable();

#if DEBUG_NO_SLEEP
            DEBUG_PRINTF("DEBUG_NO_SLEEP: simulating %ld s\n", (long)sleepSec);
            delay((uint32_t)sleepSec * 1000UL);
#else
            rtcManager.standby();
            // --- Resumes here on RTC alarm ---
#endif

            Watchdog.enable(WDT_TIMEOUT_MS);
        }

        state = STATE_GPS_SYNC;
        break;
    }

    } // end switch
}
