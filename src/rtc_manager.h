#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <Arduino.h>
#include <RTCZero.h>
#include <Adafruit_GPS.h>

/**
 * RTCManagerM0 — lightweight wrapper around RTCZero for SAMD21
 *
 * Responsibilities:
 *  - Hardware RTC initialisation
 *  - GPS-assisted time (and optional position) synchronisation via Adafruit GPS
 *  - Unix epoch / DTN epoch helpers
 *  - 10-minute slot alarm management
 */
class RTCManagerM0 {
public:
    RTCManagerM0();

    // --- Initialise hardware RTC (call once in setup) ---
    void initialize();

    // --- GPS synchronisation --------------------------------------------------
    // Syncs time (mandatory) and optionally 3D position.
    // timeoutMs == 0 → wait indefinitely.  Returns true if time was acquired.
    bool syncWithGPS(uint32_t timeoutMs, bool acquirePosition);

    // Non-blocking: feed whatever bytes are currently available on GPS_SERIAL
    // into the NMEA decoder.  Returns true (and sets the RTC) the first time a
    // valid time fix is decoded.  Caller must have already called openGPSSerial().
    bool pollGPSTime();

    // Set RTC from a Unix timestamp (seconds since 1970-01-01).
    // Also marks the manager as time-synced.
    void setRTCFromUnixEpoch(uint32_t unix);

    // --- GPS serial power management (public so fallback state can reuse them) ---
    void openGPSSerial();
    void closeGPSSerial();

    // --- Time accessors -------------------------------------------------------
    // Returns seconds elapsed since 1970-01-01 00:00:00 UTC (Unix epoch).
    // Computed from the RTCZero H:M:S + D:M:Y registers.
    uint32_t getUnixEpoch();

    // Returns milliseconds since 2000-01-01 00:00:00 UTC (DTN epoch).
    // Returns 0 if RTC has never been set.
    uint64_t getDtnTimeMs();

    uint8_t getHours();
    uint8_t getMinutes();
    uint8_t getSeconds();

    bool    isTimeSynced() const { return timeSynced_; }

    // --- GPS position cache ---------------------------------------------------
    bool  isPositionValid()  const { return positionValid_; }
    float getLatitude()      const { return latitude_; }
    float getLongitude()     const { return longitude_; }
    float getAltitude()      const { return altitude_; }
    uint8_t getGpsSatellites() const { return satellites_; }

    // --- Slot alarm -----------------------------------------------------------
    // Compute and set the RTC alarm for the next 10-min slot boundary minus
    // earlyWakeSeconds.  Attach the ISR callback before calling standby().
    void setAlarmForNextSlot(uint32_t earlyWakeSeconds);

    // Set a raw alarm time (24-h clock).
    void setAlarm(uint8_t h, uint8_t m, uint8_t s);

    // Attach ISR called when alarm fires.
    void attachAlarmCallback(voidFuncPtr cb);

    // Enter SAMD21 standby (deep sleep). Wakes on RTC alarm.
    void standby();

private:
    RTCZero      rtc_;
    Adafruit_GPS gps_;

    bool     timeSynced_;
    bool     positionValid_;
    float    latitude_;
    float    longitude_;
    float    altitude_;
    uint8_t  satellites_;

    // Set RTC registers from the currently parsed Adafruit_GPS fields
    void setRTCFromAdafruit();

    // Calendar helpers
    static uint32_t toUnixEpoch(uint8_t year2k, uint8_t month, uint8_t day,
                                 uint8_t hour, uint8_t minute, uint8_t second);

    // Calendar helper (used by setRTCFromUnixEpoch)
    static void unixToCalendar(uint32_t unix,
                               uint8_t& year2k, uint8_t& month, uint8_t& day,
                               uint8_t& hour,   uint8_t& minute, uint8_t& second);
};

// Global instance (defined in rtc_manager.cpp)
extern RTCManagerM0 rtcManager;

#endif // RTC_MANAGER_H
