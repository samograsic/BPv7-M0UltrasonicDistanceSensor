#include "sensor.h"
#include "config.h"
#include <Wire.h>
#include <math.h>
#include <Adafruit_SleepyDog.h>

SensorManager sensorManager;

// ---------------------------------------------------------------------------
// I2CXL-MaxSonar protocol (MaxBotix application note)
//
//  1. Write 0x51 to the sensor  → initiates ranging pulse
//  2. Wait 100 ms               → sensor echoes and computes range
//  3. Read 2 bytes              → high byte | low byte = range in cm
//
// I2C address: 0x70 (7-bit)
// Valid range: 20–765 cm (model dependent)
// ---------------------------------------------------------------------------

bool SensorManager::initialize() {
    Wire.begin();
    // Quick presence check — try a write
    Wire.beginTransmission(SONAR_I2C_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        DEBUG_PRINTF("Sonar: I2C NACK at 0x%02X (err=%d)\n", SONAR_I2C_ADDR, err);
        return false;
    }
    DEBUG_PRINTF("Sonar: found at 0x%02X\n", SONAR_I2C_ADDR);
    return true;
}

uint16_t SensorManager::takeSingleMeasurement() {
    // Start ranging command
    Wire.beginTransmission(SONAR_I2C_ADDR);
    Wire.write(0x51);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        DEBUG_PRINTF("Sonar: TX err %d\n", err);
        return 0;
    }

    delay(100); // wait for measurement to complete

    // Read 2 bytes (big-endian cm)
    uint8_t n = Wire.requestFrom((uint8_t)SONAR_I2C_ADDR, (uint8_t)2);
    if (n < 2) {
        DEBUG_PRINTLN("Sonar: short read");
        return 0;
    }

    uint16_t high = Wire.read();
    uint16_t low  = Wire.read();
    return (high << 8) | low;
}

// ---------------------------------------------------------------------------
// measure() — collect N samples and apply outlier-rejection statistics
// ---------------------------------------------------------------------------
SonarResult SensorManager::measure() {
    uint16_t raw[SONAR_NUM_SAMPLES];
    uint8_t  n = 0;

    DEBUG_PRINTF("Sonar: collecting %d samples...\n", SONAR_NUM_SAMPLES);

    for (int i = 0; i < SONAR_NUM_SAMPLES; i++) {
        uint16_t d = takeSingleMeasurement();
        if (d >= SONAR_MIN_CM && d <= SONAR_MAX_CM) {
            raw[n++] = d;
            DEBUG_PRINTF("  [%d] %d cm\n", i, d);
        } else {
            DEBUG_PRINTF("  [%d] %d cm REJECTED (out of range)\n", i, d);
        }
        if (i < SONAR_NUM_SAMPLES - 1) delay(SONAR_SAMPLE_DELAY);
        Watchdog.reset();  // 100 samples × ~250 ms = ~25 s; must pet WDT each sample
    }

    SonarResult result = {0, 0, 0, false, 0.0f};
    if (n < 3) {
        DEBUG_PRINTF("Sonar: only %d valid samples — marking invalid\n", n);
        return result;
    }

    // --- Pass 1: mean of all valid samples ---
    float sum = 0;
    for (int i = 0; i < n; i++) sum += raw[i];
    float mean = sum / n;

    // --- Standard deviation ---
    float sqSum = 0;
    for (int i = 0; i < n; i++) {
        float d = raw[i] - mean;
        sqSum += d * d;
    }
    float stdDev = sqrtf(sqSum / n);

    // --- Pass 2: reject outliers (> SONAR_OUTLIER_SIGMA × σ from mean) ---
    float filtSum = 0;
    uint8_t valid = 0;
    for (int i = 0; i < n; i++) {
        float dev = fabsf((float)raw[i] - mean);
        if (dev <= SONAR_OUTLIER_SIGMA * stdDev) {
            filtSum += raw[i];
            valid++;
        } else {
            DEBUG_PRINTF("  raw[%d]=%d cm OUTLIER (dev=%.1f > %.1f*σ)\n",
                         i, raw[i], dev, SONAR_OUTLIER_SIGMA);
        }
    }

    float filtMean = (valid > 0) ? filtSum / valid : mean;

    // --- Recompute std dev over accepted samples ---
    float filtSqSum = 0;
    float filtCount  = 0;
    for (int i = 0; i < n; i++) {
        float dev = fabsf((float)raw[i] - mean);
        if (dev <= SONAR_OUTLIER_SIGMA * stdDev) {
            float d = raw[i] - filtMean;
            filtSqSum += d * d;
            filtCount++;
        }
    }
    float filtStdDev = (filtCount > 1) ? sqrtf(filtSqSum / filtCount) : 0;

    // --- Temperature compensation for speed of sound ---
    // v(T) = 331.3 + 0.606·T m/s;  sensor calibrated at 20 °C → v_ref = 343.42 m/s
    float tempC      = readInternalTemperature();
    float correction = (331.3f + 0.606f * tempC) / (331.3f + 0.606f * 20.0f);

    float corrMean   = filtMean * correction;
    float corrStdDev = filtStdDev * correction;

    result.meanCm    = (uint16_t)(corrMean + 0.5f);
    result.stdDevCm  = (corrStdDev > 255.0f) ? 255 : (uint8_t)(corrStdDev + 0.5f);
    result.validCount = valid;
    result.valid     = (valid >= 3);
    result.tempC     = tempC;

    DEBUG_PRINTF("Sonar: raw=%.1f cm  temp=%.1f C  corr=%.4f  adj=%d cm  σ=%d cm  valid=%d/%d\n",
                 filtMean, tempC, correction, result.meanCm, result.stdDevCm, valid, n);
    return result;
}

// ---------------------------------------------------------------------------
// Battery voltage — Feather M0 built-in A7 divider (2:1, 3.3 V ref, 10-bit)
// ---------------------------------------------------------------------------
float SensorManager::readBatteryVoltage() {
    // Two samples for stability
    analogRead(VBAT_PIN);          // discard first (reference settling)
    float raw = (float)analogRead(VBAT_PIN);
    float vbat = raw * 2.0f * 3.3f / 1024.0f;
    return vbat;
}

// ---------------------------------------------------------------------------
// Internal SAMD21 temperature sensor
// Based on Adafruit and Microchip datasheet method
// ---------------------------------------------------------------------------
float SensorManager::readInternalTemperature() {
    // Enable temperature sensor
    SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN;

    // Save ADC state
    uint16_t oldCtrlB    = ADC->CTRLB.reg;
    uint16_t oldSampCtrl = ADC->SAMPCTRL.reg;
    uint8_t  oldGain     = ADC->INPUTCTRL.bit.GAIN;
    uint8_t  oldRefSel   = ADC->REFCTRL.bit.REFSEL;

    ADC->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_PRESCALER_DIV256;
    while (ADC->STATUS.bit.SYNCBUSY);
    ADC->SAMPCTRL.reg = 0x3F;
    while (ADC->STATUS.bit.SYNCBUSY);
    ADC->INPUTCTRL.bit.GAIN  = ADC_INPUTCTRL_GAIN_1X_Val;
    ADC->REFCTRL.bit.REFSEL  = ADC_REFCTRL_REFSEL_INT1V_Val;
    while (ADC->STATUS.bit.SYNCBUSY);
    ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_TEMP_Val;
    ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY);
    ADC->SWTRIG.bit.START = 1;
    ADC->INTFLAG.bit.RESRDY = 1; // clear
    while (ADC->STATUS.bit.SYNCBUSY);
    ADC->SWTRIG.bit.START = 1;
    while (!ADC->INTFLAG.bit.RESRDY);
    int32_t adcVal = ADC->RESULT.reg;

    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY);

    // Restore
    ADC->CTRLB.reg = oldCtrlB;
    while (ADC->STATUS.bit.SYNCBUSY);
    ADC->SAMPCTRL.reg = oldSampCtrl;
    while (ADC->STATUS.bit.SYNCBUSY);
    ADC->INPUTCTRL.bit.GAIN = oldGain;
    ADC->REFCTRL.bit.REFSEL = oldRefSel;
    while (ADC->STATUS.bit.SYNCBUSY);

    // Factory calibration fuses
    uint8_t  roomInt  = (*(uint32_t*)FUSES_ROOM_TEMP_VAL_INT_ADDR
                         & FUSES_ROOM_TEMP_VAL_INT_Msk) >> FUSES_ROOM_TEMP_VAL_INT_Pos;
    uint8_t  roomDec  = (*(uint32_t*)FUSES_ROOM_TEMP_VAL_DEC_ADDR
                         & FUSES_ROOM_TEMP_VAL_DEC_Msk) >> FUSES_ROOM_TEMP_VAL_DEC_Pos;
    int32_t  roomADC  = ((*(uint32_t*)FUSES_ROOM_ADC_VAL_ADDR
                          & FUSES_ROOM_ADC_VAL_Msk) >> FUSES_ROOM_ADC_VAL_Pos);
    int32_t  roomTemp = (int32_t)roomInt * 1000 + (int32_t)roomDec * 100;

    uint8_t  hotInt   = (*(uint32_t*)FUSES_HOT_TEMP_VAL_INT_ADDR
                         & FUSES_HOT_TEMP_VAL_INT_Msk) >> FUSES_HOT_TEMP_VAL_INT_Pos;
    uint8_t  hotDec   = (*(uint32_t*)FUSES_HOT_TEMP_VAL_DEC_ADDR
                         & FUSES_HOT_TEMP_VAL_DEC_Msk) >> FUSES_HOT_TEMP_VAL_DEC_Pos;
    int32_t  hotADC   = ((*(uint32_t*)FUSES_HOT_ADC_VAL_ADDR
                          & FUSES_HOT_ADC_VAL_Msk) >> FUSES_HOT_ADC_VAL_Pos);
    int32_t  hotTemp  = (int32_t)hotInt * 1000 + (int32_t)hotDec * 100;

    // Linear interpolation
    int32_t temp1000 = roomTemp
        + ((hotTemp - roomTemp) * (adcVal - roomADC)) / (hotADC - roomADC);
    return (float)temp1000 / 1000.0f;
}

// ---------------------------------------------------------------------------
// CBOR encoding helpers (same wire format as StatusBundleEncoder)
// ---------------------------------------------------------------------------

static bool cborUint(uint64_t v, uint8_t* p, size_t rem, size_t* w) {
    if (v <= 23) {
        if (rem < 1) return false;
        p[0] = (uint8_t)v; *w = 1;
    } else if (v <= 0xFF) {
        if (rem < 2) return false;
        p[0] = 0x18; p[1] = (uint8_t)v; *w = 2;
    } else if (v <= 0xFFFF) {
        if (rem < 3) return false;
        p[0] = 0x19; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)v; *w = 3;
    } else if (v <= 0xFFFFFFFFUL) {
        if (rem < 5) return false;
        p[0] = 0x1A;
        p[1] = (uint8_t)(v >> 24); p[2] = (uint8_t)(v >> 16);
        p[3] = (uint8_t)(v >>  8); p[4] = (uint8_t)v;
        *w = 5;
    } else {
        if (rem < 9) return false;
        p[0] = 0x1B;
        p[1] = (uint8_t)(v >> 56); p[2] = (uint8_t)(v >> 48);
        p[3] = (uint8_t)(v >> 40); p[4] = (uint8_t)(v >> 32);
        p[5] = (uint8_t)(v >> 24); p[6] = (uint8_t)(v >> 16);
        p[7] = (uint8_t)(v >>  8); p[8] = (uint8_t)v;
        *w = 9;
    }
    return true;
}

static bool cborFloat(float v, uint8_t* p, size_t rem, size_t* w) {
    if (rem < 5) { *w = 0; return false; }
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    p[0] = 0xFA;                         // single-precision float header
    p[1] = (uint8_t)(bits >> 24); p[2] = (uint8_t)(bits >> 16);
    p[3] = (uint8_t)(bits >>  8); p[4] = (uint8_t)bits;
    *w = 5; return true;
}

static bool cborNull(uint8_t* p, size_t rem, size_t* w) {
    if (rem < 1) return false;
    p[0] = 0xF6; *w = 1; return true;
}

static bool cborMapHeader(uint8_t n, uint8_t* p, size_t rem, size_t* w) {
    if (n <= 23) {
        if (rem < 1) return false;
        p[0] = 0xA0 | n; *w = 1;
    } else {
        if (rem < 2) return false;
        p[0] = 0xB8; p[1] = n; *w = 2;
    }
    return true;
}

// Convenience: write key (uint) then dispatch value or null in one expression.
// All helpers must be called via macros so early-return propagates correctly.
#define CBOR_KEY(k)           do { if (!cborUint((k), buf+off, maxLen-off, &w)) return false; off += w; } while(0)
#define CBOR_UINT(v)          do { if (!cborUint((v), buf+off, maxLen-off, &w)) return false; off += w; } while(0)
#define CBOR_FLOAT(v)         do { if (!cborFloat((v), buf+off, maxLen-off, &w)) return false; off += w; } while(0)
#define CBOR_BOOL(v)          do { if (!cborBool((v), buf+off, maxLen-off, &w)) return false; off += w; } while(0)
#define CBOR_FLOAT_OR_NULL(cond, v) \
    do { if (cond) { if (!cborFloat((v), buf+off, maxLen-off, &w)) return false; } \
         else      { if (!cborNull(buf+off, maxLen-off, &w)) return false; }       \
         off += w; } while(0)

// ---------------------------------------------------------------------------
// buildCBORPayload
//
// Encodes a 13-entry CBOR map into buf[0..maxLen).  Returns false only if the
// buffer is smaller than ~80 bytes (128 bytes is always sufficient).
//
// Key mapping (compatible with StatusBundleEncoder where applicable):
//   1=station_id  2=timestamp_ms  4=cpu_temperature  5=battery_voltage
//  13=gps_lat    14=gps_lon      15=gps_alt
//  22=sonar_distance_cm  23=sonar_stddev_cm  24=sonar_samples
//  25=sonar_delta_cm     26=gps_satellites   27=gps_cached
// ---------------------------------------------------------------------------
bool SensorManager::buildCBORPayload(
    uint8_t* buf, size_t maxLen, size_t* outLen,
    uint32_t unixTs,
    float lat, float lon, float altM,
    uint8_t /*sats*/, bool gpsValid, bool gpsCached,
    const SonarResult& sonar,
    int16_t deltaCm, bool deltaValid)
{
    float    vbat    = readBatteryVoltage();
    size_t   off     = 0;
    size_t   w       = 0;
    bool     hasPosData = gpsValid || gpsCached;

    // DTN epoch = 2000-01-01 00:00:00 UTC = Unix 946684800
    uint64_t dtnMs = (unixTs > 946684800UL)
                   ? ((uint64_t)(unixTs - 946684800UL) * 1000ULL)
                   : 0ULL;

    // Map header — 10 key-value pairs
    if (!cborMapHeader(10, buf + off, maxLen - off, &w)) return false;
    off += w;

    CBOR_KEY(1);  CBOR_UINT((uint64_t)NODE_NUMBER);         // station_id
    CBOR_KEY(2);  CBOR_UINT(dtnMs);                         // timestamp_ms
    CBOR_KEY(4);  CBOR_FLOAT(sonar.tempC);                  // cpu_temperature
    CBOR_KEY(5);  CBOR_FLOAT(vbat);                         // battery_voltage

    CBOR_KEY(13); CBOR_FLOAT_OR_NULL(hasPosData, lat);      // gps_latitude
    CBOR_KEY(14); CBOR_FLOAT_OR_NULL(hasPosData, lon);      // gps_longitude
    CBOR_KEY(15); CBOR_FLOAT_OR_NULL(hasPosData, altM);     // gps_altitude

    CBOR_KEY(22); CBOR_FLOAT_OR_NULL(sonar.valid, (float)sonar.meanCm);    // sonar_distance_cm
    CBOR_KEY(23); CBOR_FLOAT_OR_NULL(sonar.valid, (float)sonar.stdDevCm);  // sonar_stddev_cm
    CBOR_KEY(25); CBOR_FLOAT_OR_NULL(deltaValid, (float)deltaCm);          // sonar_delta_cm

    *outLen = off;

    DEBUG_PRINTF("Payload: %d B CBOR  dist=%s cm  delta=%s cm  vbat=%.2f V\n",
                 (int)off,
                 sonar.valid  ? String(sonar.meanCm).c_str()  : "null",
                 deltaValid   ? String((int)deltaCm).c_str()  : "null",
                 vbat);

    return true;
}

#undef CBOR_KEY
#undef CBOR_UINT
#undef CBOR_FLOAT
#undef CBOR_FLOAT_OR_NULL
