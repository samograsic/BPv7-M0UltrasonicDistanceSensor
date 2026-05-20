#ifndef CPU_UTILS_H
#define CPU_UTILS_H

// cpu_utils.h — M0 stub
// Provides getTrueRandom() using ADC noise + SysTick jitter on SAMD21.
// The SAMD21 has no TRNG peripheral; this approximation is sufficient
// for LoRa collision-avoidance backoff and HMAC nonces.

#include <Arduino.h>

inline uint32_t getTrueRandom() {
    // XOR multiple entropy sources: ADC noise, SysTick, millis
    uint32_t seed = (uint32_t)analogRead(A0) ^ SysTick->VAL ^ millis();
    // Xorshift to spread entropy
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}

#endif // CPU_UTILS_H
