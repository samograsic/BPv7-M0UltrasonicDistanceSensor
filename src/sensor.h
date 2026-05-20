#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// SonarResult — output of one measurement cycle (10 samples → filtered stats)
// ---------------------------------------------------------------------------
struct SonarResult {
    uint16_t meanCm;        // Temperature-compensated filtered mean distance (cm)
    uint8_t  stdDevCm;      // Standard deviation of valid samples (cm, capped 255)
    uint8_t  validCount;    // Number of accepted samples (after outlier rejection)
    bool     valid;         // false if < 3 valid samples obtained
    float    tempC;         // CPU temperature used for speed-of-sound correction (°C)
};

// ---------------------------------------------------------------------------
// Sensor CBOR payload key assignments
//
// Keys 1-21 are shared with the base-station StatusBundleEncoder so the
// gateway can decode both node types with the same schema.
// Keys 22-27 are sensor-specific extensions.
//
//  key= 1  station_id         uint    - NODE_NUMBER
//  key= 2  timestamp_ms       uint    - ms since DTN epoch (2000-01-01), 0 if unknown
//  key= 4  cpu_temperature    float   - °C (also used for speed-of-sound correction)
//  key= 5  battery_voltage    float   - V
//  key=13  gps_latitude       float|null
//  key=14  gps_longitude      float|null
//  key=15  gps_altitude       float|null  (metres)
//  key=22  sonar_distance_cm  float|null  (temperature-compensated cm)
//  key=23  sonar_stddev_cm    float|null  (cm)
//  key=25  sonar_delta_cm     float|null  - signed delta vs previous reading
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SensorManager — drives I2CXL-MaxSonar and battery/temperature readback
// ---------------------------------------------------------------------------
class SensorManager {
public:
    SensorManager() = default;

    // Call once in setup() after Wire.begin()
    bool initialize();

    // Take SONAR_NUM_SAMPLES measurements, apply statistical filtering.
    SonarResult measure();

    // Read battery voltage from Feather M0 A7 divider (3.3 V ref, 10-bit ADC)
    float readBatteryVoltage();

    // Read SAMD21 internal temperature sensor (°C)
    float readInternalTemperature();

    // Encode measurement data as a CBOR map into caller-supplied buffer.
    // Returns false only if the buffer is too small (128 bytes is always enough).
    bool buildCBORPayload(
        uint8_t* buf, size_t maxLen, size_t* outLen,
        uint32_t unixTs,
        float    lat, float lon, float altM,
        uint8_t  satellites, bool gpsValid, bool gpsCached,
        const SonarResult& sonar,
        int16_t  deltaCm, bool deltaValid);

private:
    // Send ranging command and read result (single sample, cm)
    // Returns 0 on I2C error or out-of-range.
    uint16_t takeSingleMeasurement();
};

// Global instance (defined in sensor.cpp)
extern SensorManager sensorManager;

#endif // SENSOR_H
