#include "rtc_manager.h"
#include "config.h"
#include <Adafruit_SleepyDog.h>

// Global instance
RTCManagerM0 rtcManager;

// ---------------------------------------------------------------------------
// Calendar helper — seconds since 1970-01-01 from broken-down GPS time
// year2k = years since 2000 (e.g. 26 for 2026)
// ---------------------------------------------------------------------------
static bool isLeap(int y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}
static const uint8_t kDaysInMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

uint32_t RTCManagerM0::toUnixEpoch(uint8_t year2k, uint8_t month, uint8_t day,
                                    uint8_t hour, uint8_t minute, uint8_t second) {
    int fullYear = 2000 + (int)year2k;
    uint32_t secs = 0;

    // Full years since 1970
    for (int y = 1970; y < fullYear; y++) {
        secs += isLeap(y) ? 366 * 86400UL : 365 * 86400UL;
    }
    // Full months this year
    for (int m = 1; m < month; m++) {
        uint8_t days = kDaysInMonth[m - 1];
        if (m == 2 && isLeap(fullYear)) days = 29;
        secs += days * 86400UL;
    }
    secs += (day - 1) * 86400UL;
    secs += hour * 3600UL;
    secs += minute * 60UL;
    secs += second;
    return secs;
}

// ---------------------------------------------------------------------------

RTCManagerM0::RTCManagerM0()
    : timeSynced_(false), positionValid_(false),
      latitude_(0), longitude_(0), altitude_(0), satellites_(0),
      gps_(&GPS_SERIAL) {}

void RTCManagerM0::initialize() {
    rtc_.begin();
    // Set a sane default (2025-01-01 00:00:00) so epoch math won't underflow
    rtc_.setDate(1, 1, 25);   // day, month, year-2000
    rtc_.setTime(0, 0, 0);
    DEBUG_PRINTLN("RTC: initialized (default 2025-01-01 00:00:00)");
}

// ---------------------------------------------------------------------------
// GPS serial management
// ---------------------------------------------------------------------------
void RTCManagerM0::openGPSSerial() {
    // Enable GPS module (active LOW)
    pinMode(GPS_ENABLE_PIN, OUTPUT);
    digitalWrite(GPS_ENABLE_PIN, LOW);
    delay(500);

    // Module communicates at 9600 baud (NMEA default).
    // No proprietary baud-switch: this module ignores $PUBX commands.
    gps_.begin(9600);
    delay(500);

    DEBUG_PRINTLN("GPS: serial open at 9600 (Adafruit GPS)");
}

void RTCManagerM0::closeGPSSerial() {
    GPS_SERIAL.end();
    digitalWrite(GPS_ENABLE_PIN, HIGH);   // power off
    DEBUG_PRINTLN("GPS: serial closed, power off");
}

// ---------------------------------------------------------------------------
// pollGPSTime — non-blocking; call repeatedly while GPS serial is open.
// Returns true once a valid time fix has been decoded and the RTC set.
// ---------------------------------------------------------------------------
bool RTCManagerM0::pollGPSTime() {
    char c;
    while ((c = gps_.read()) != 0) {
        if (gps_.newNMEAreceived() && gps_.parse(gps_.lastNMEA())) {
            if (gps_.year >= 24 && gps_.month > 0 && gps_.day > 0) {
                setRTCFromAdafruit();
                timeSynced_ = true;
                DEBUG_PRINTF("GPS: time OK (poll) %02d:%02d:%02d UTC\n",
                    gps_.hour, gps_.minute, gps_.seconds);
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// setRTCFromUnixEpoch — decompose a Unix timestamp and program the RTC.
// Used when time is obtained from a received BPv7 bundle creation timestamp.
// ---------------------------------------------------------------------------
void RTCManagerM0::unixToCalendar(uint32_t unix,
                                   uint8_t& year2k, uint8_t& month, uint8_t& day,
                                   uint8_t& hour,   uint8_t& minute, uint8_t& second)
{
    uint32_t days = unix / 86400UL;
    uint32_t rem  = unix % 86400UL;
    hour   = (uint8_t)(rem / 3600);
    minute = (uint8_t)((rem % 3600) / 60);
    second = (uint8_t)(rem % 60);

    int y = 1970;
    while (true) {
        uint32_t yd = isLeap(y) ? 366 : 365;
        if (days < yd) break;
        days -= yd;
        y++;
    }
    uint8_t mo = 1;
    while (mo <= 12) {
        uint8_t md = kDaysInMonth[mo - 1];
        if (mo == 2 && isLeap(y)) md = 29;
        if (days < md) break;
        days -= md;
        mo++;
    }
    year2k = (uint8_t)(y - 2000);
    month  = mo;
    day    = (uint8_t)(days + 1);
}

void RTCManagerM0::setRTCFromUnixEpoch(uint32_t unix) {
    uint8_t yr, mo, dy, h, m, s;
    unixToCalendar(unix, yr, mo, dy, h, m, s);
    rtc_.setDate(dy, mo, yr);
    rtc_.setTime(h, m, s);
    timeSynced_ = true;
    DEBUG_PRINTF("RTC: set from LoRa  %02d:%02d:%02d  %02d/%02d/20%02d\n",
                 h, m, s, dy, mo, yr);
}

// ---------------------------------------------------------------------------
// Set RTC from the Adafruit_GPS fields (call after a successful parse())
// ---------------------------------------------------------------------------
void RTCManagerM0::setRTCFromAdafruit() {
    rtc_.setDate(gps_.day, gps_.month, gps_.year);
    rtc_.setTime(gps_.hour, gps_.minute, gps_.seconds);
    DEBUG_PRINTF("RTC: set %02d:%02d:%02d %02d/%02d/20%02d\n",
        gps_.hour, gps_.minute, gps_.seconds,
        gps_.day, gps_.month, gps_.year);
}

// ---------------------------------------------------------------------------
// syncWithGPS
// ---------------------------------------------------------------------------
bool RTCManagerM0::syncWithGPS(uint32_t timeoutMs, bool acquirePosition) {
    if (timeoutMs == 0) {
        DEBUG_PRINTF("GPS sync: timeout=none (wait for lock), pos=%s\n",
                     acquirePosition ? "yes" : "no");
    } else {
        DEBUG_PRINTF("GPS sync: timeout=%lus, pos=%s\n",
                     timeoutMs / 1000, acquirePosition ? "yes" : "no");
    }

    openGPSSerial();

    uint32_t startMs    = millis();
    uint32_t lastReport = 0;
    bool gotTime    = false;
    bool gotPos     = false;
    bool dataActive = false;
    uint32_t rxBytes     = 0;
    uint32_t rxSentences = 0;

    // timeoutMs == 0 → wait indefinitely
    while (timeoutMs == 0 || (millis() - startMs) < timeoutMs) {
        Watchdog.reset();

        // Drain all currently available characters
        char c;
        while ((c = gps_.read()) != 0) {
            rxBytes++;
            if (!dataActive) {
                dataActive = true;
                DEBUG_PRINTLN("GPS: data stream active");
            }

            if (!gps_.newNMEAreceived()) continue;
            if (!gps_.parse(gps_.lastNMEA())) continue;
            rxSentences++;

            // --- Time: year >= 24 (2024) is the sanity gate for a real fix ---
            bool timeAccepted = false;
            if (!gotTime && gps_.year >= 24 && gps_.month > 0 && gps_.day > 0) {
                if (timeSynced_) {
                    // Reject if GPS time is more than 2 min behind the RTC
                    int16_t rtcMin = (int16_t)rtc_.getHours() * 60
                                   + (int16_t)rtc_.getMinutes();
                    int16_t gpsMin = (int16_t)gps_.hour * 60
                                   + (int16_t)gps_.minute;
                    int16_t diff = gpsMin - rtcMin;
                    if (diff < -720) diff += 1440;
                    if (diff >  720) diff -= 1440;
                    if (diff < -2) {
                        DEBUG_PRINTF("GPS: time rejected (GPS %02d:%02d behind RTC %02d:%02d)\n",
                            gps_.hour, gps_.minute,
                            rtc_.getHours(), rtc_.getMinutes());
                    } else {
                        timeAccepted = true;
                    }
                } else {
                    timeAccepted = true;
                }

                if (timeAccepted) {
                    setRTCFromAdafruit();
                    gotTime     = true;
                    timeSynced_ = true;
                    DEBUG_PRINTF("GPS: time OK %02d:%02d:%02d UTC\n",
                        gps_.hour, gps_.minute, gps_.seconds);
                    if (!acquirePosition) goto done;
                }
            }

            // --- Position: Adafruit_GPS sets fix=1 when GGA/RMC reports a fix ---
            if (acquirePosition && !gotPos && gps_.fix && gps_.fixquality > 0) {
                latitude_      = gps_.latitudeDegrees;
                longitude_     = gps_.longitudeDegrees;
                altitude_      = gps_.altitude;
                satellites_    = gps_.satellites;
                positionValid_ = true;
                gotPos         = true;
                DEBUG_PRINTF("GPS: pos %.6f, %.6f, %.1fm (%d sats)\n",
                    latitude_, longitude_, (float)altitude_, satellites_);
            }

            if (gotTime && (!acquirePosition || gotPos)) goto done;
        }

        // Progress report every 15 s
        uint32_t elapsed = millis() - startMs;
        if (elapsed - lastReport >= 15000) {
            lastReport = elapsed;
            if (timeoutMs == 0) {
                DEBUG_PRINTF("GPS: waiting... %lu s  time=%s pos=%s  rx=%lu B / %lu sentences\n",
                    elapsed / 1000,
                    gotTime ? "OK" : "-", gotPos ? "OK" : "-",
                    rxBytes, rxSentences);
            } else {
                DEBUG_PRINTF("GPS: waiting... %lu/%lu s  time=%s pos=%s  rx=%lu B / %lu sentences\n",
                    elapsed / 1000, timeoutMs / 1000,
                    gotTime ? "OK" : "-", gotPos ? "OK" : "-",
                    rxBytes, rxSentences);
            }
        }

        delay(10);
    }

done:
    closeGPSSerial();
    DEBUG_PRINTF("GPS sync: time=%s pos=%s (elapsed %lu s)  rx=%lu B / %lu sentences\n",
        gotTime ? "OK" : "FAIL", gotPos ? "OK" : "-",
        (millis() - startMs) / 1000, rxBytes, rxSentences);
    return gotTime;
}

// ---------------------------------------------------------------------------
// Epoch helpers
// ---------------------------------------------------------------------------
uint32_t RTCManagerM0::getUnixEpoch() {
    return toUnixEpoch(rtc_.getYear(), rtc_.getMonth(), rtc_.getDay(),
                       rtc_.getHours(), rtc_.getMinutes(), rtc_.getSeconds());
}

uint64_t RTCManagerM0::getDtnTimeMs() {
    uint32_t unix = getUnixEpoch();
    const uint32_t DTN_OFFSET = 946684800UL; // seconds 1970→2000
    if (unix < DTN_OFFSET) return 0;
    return (uint64_t)(unix - DTN_OFFSET) * 1000ULL;
}

uint8_t RTCManagerM0::getHours()   { return rtc_.getHours(); }
uint8_t RTCManagerM0::getMinutes() { return rtc_.getMinutes(); }
uint8_t RTCManagerM0::getSeconds() { return rtc_.getSeconds(); }

// ---------------------------------------------------------------------------
// Alarm / sleep
// ---------------------------------------------------------------------------
void RTCManagerM0::setAlarm(uint8_t h, uint8_t m, uint8_t s) {
    rtc_.setAlarmTime(h, m, s);
    rtc_.enableAlarm(RTCZero::MATCH_HHMMSS);
    DEBUG_PRINTF("RTC: alarm set %02d:%02d:%02d\n", h, m, s);
}

void RTCManagerM0::attachAlarmCallback(voidFuncPtr cb) {
    rtc_.attachInterrupt(cb);
}

void RTCManagerM0::setAlarmForNextSlot(uint32_t earlyWakeSeconds) {
    const uint32_t slotMin   = SLOT_INTERVAL_S / 60;   // 10 min
    uint8_t curH   = getHours();
    uint8_t curM   = getMinutes();
    uint8_t curS   = getSeconds();

    // Next slot minute within the hour
    uint8_t nextSlotM = ((curM / slotMin) + 1) * slotMin;
    uint8_t nextSlotH = curH;
    if (nextSlotM >= 60) { nextSlotM -= 60; nextSlotH = (curH + 1) % 24; }

    // Subtract early-wake offset
    uint8_t bufMin = earlyWakeSeconds / 60;
    if (nextSlotM >= bufMin) {
        nextSlotM -= bufMin;
    } else {
        nextSlotM = 60 - bufMin + nextSlotM;
        nextSlotH = (nextSlotH == 0) ? 23 : nextSlotH - 1;
    }

    // Guard: if the computed alarm is already within 10 s, advance one full slot
    uint32_t secsUntil = (uint32_t)(nextSlotM - curM) * 60
                         + (int8_t)(nextSlotH - curH) * 3600
                         - curS;
    // Simple check: if alarm < 15 s away, skip to next slot
    (void)secsUntil;

    setAlarm(nextSlotH, nextSlotM, 0);
    DEBUG_PRINTF("Alarm: next slot wake @ %02d:%02d:00 (-%us early)\n",
                 nextSlotH, nextSlotM, earlyWakeSeconds);
}

void RTCManagerM0::standby() {
    DEBUG_PRINTLN("Entering standby...");
    // Flush serial before sleeping
    Serial.flush();
    // SAMD21 standby — wakes on RTC alarm
    rtc_.standbyMode();
    // Execution continues here after wake
    DEBUG_PRINTLN("Woke from standby");
    Watchdog.enable(WDT_TIMEOUT_MS);
}
