#include "LoRaSemDriver.h"

// Static singleton for ISR access
LoRaSemDriver* LoRaSemDriver::instance = nullptr;

LoRaSemDriver::LoRaSemDriver()
    : initialized(false), packetReceived(false), lastRssi(0), lastSnr(0.0f), receiveCallback(nullptr) {
    instance = this;  // Set singleton instance
}

LoRaSemDriver::~LoRaSemDriver() {
    if (initialized) {
        LoRa.end();
    }
    instance = nullptr;
}

int LoRaSemDriver::init(int cs, int dio0, int rst, int dio1,
                         float frequency, float bandwidth,
                         uint8_t spreadingFactor, uint8_t codingRate,
                         int8_t txPower, uint8_t syncWord, uint8_t preambleLength) {
    Serial.println("=== LoRaSemDriver: Initializing ===");
    Serial.printf("  Frequency: %.2f MHz\n", frequency);
    Serial.printf("  Bandwidth: %.1f kHz\n", bandwidth);
    Serial.printf("  Spreading Factor: %d\n", spreadingFactor);
    Serial.printf("  Coding Rate: 4/%d\n", codingRate);
    Serial.printf("  TX Power: %d dBm\n", txPower);
    Serial.printf("  Sync Word: 0x%02X\n", syncWord);
    Serial.printf("  Preamble Length: %d symbols\n", preambleLength);
    Serial.printf("  Pins - CS:%d, DIO0:%d, RST:%d\n", cs, dio0, rst);

    // Configure pins
    LoRa.setPins(cs, rst, dio0);

    // Begin with frequency (in Hz, LoRaSem expects long)
    long freq_hz = (long)(frequency * 1E6);
    if (!LoRa.begin(freq_hz)) {
        Serial.println("LoRaSemDriver: begin() failed!");
        return RADIOLIB_ERR_CHIP_NOT_FOUND;
    }

    // Configure radio parameters
    LoRa.setSpreadingFactor(spreadingFactor);
    LoRa.setPreambleLength(preambleLength);
    LoRa.setSignalBandwidth((long)(bandwidth * 1000.0f));  // Convert kHz to Hz
    LoRa.setCodingRate4(codingRate);
    LoRa.setTxPower(txPower);
    LoRa.setSyncWord(syncWord);
    LoRa.enableCrc();

    Serial.println("LoRaSemDriver: Initialization successful!");
    Serial.printf("LoRaSemDriver: Final config - Freq: %.2f MHz, BW: %.1f kHz, SF: %d, CR: 4/%d, Preamble: %d\n",
                  frequency, bandwidth, spreadingFactor, codingRate, preambleLength);
    Serial.printf("LoRaSemDriver: TX Power: %d dBm, Sync Word: 0x%02X, CRC: Enabled\n",
                  txPower, syncWord);

    initialized = true;
    return RADIOLIB_ERR_NONE;
}

int LoRaSemDriver::transmit(const uint8_t* data, size_t len) {
    if (!initialized) {
        return RADIOLIB_ERR_CHIP_NOT_FOUND;
    }

    // Begin packet
    if (!LoRa.beginPacket()) {
        Serial.println("LoRaSemDriver: beginPacket() failed");
        return -1;
    }

    // Write data
    size_t written = LoRa.write(data, len);
    if (written != len) {
        Serial.printf("LoRaSemDriver: write() failed (wrote %d/%d bytes)\n", written, len);
        return -1;
    }

    // End packet (blocking transmission)
    if (!LoRa.endPacket()) {
        Serial.println("LoRaSemDriver: endPacket() failed");
        return -1;
    }

    return RADIOLIB_ERR_NONE;
}

int LoRaSemDriver::receive(uint8_t* buffer, size_t max_len, size_t* out_len, uint32_t timeout_ms) {
    if (!initialized) {
        return RADIOLIB_ERR_CHIP_NOT_FOUND;
    }

    // Check if packet is already available (from ISR)
    if (packetReceived) {
        packetReceived = false;  // Clear flag

        // Read packet
        int packetSize = LoRa.available();
        if (packetSize > 0 && packetSize <= (int)max_len) {
            for (int i = 0; i < packetSize; i++) {
                buffer[i] = LoRa.read();
            }
            *out_len = packetSize;

            // Get RSSI and SNR
            lastRssi = LoRa.packetRssi();
            lastSnr = LoRa.packetSnr();

            return RADIOLIB_ERR_NONE;
        } else {
            *out_len = 0;
            return -1;
        }
    }

    // No packet available yet, poll with timeout
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        if (packetReceived) {
            packetReceived = false;  // Clear flag

            int packetSize = LoRa.available();
            if (packetSize > 0 && packetSize <= (int)max_len) {
                for (int i = 0; i < packetSize; i++) {
                    buffer[i] = LoRa.read();
                }
                *out_len = packetSize;

                // Get RSSI and SNR
                lastRssi = LoRa.packetRssi();
                lastSnr = LoRa.packetSnr();

                return RADIOLIB_ERR_NONE;
            } else {
                *out_len = 0;
                return -1;
            }
        }
        delay(10);
    }

    // Timeout
    *out_len = 0;
    return RADIOLIB_ERR_RX_TIMEOUT;
}

int LoRaSemDriver::startReceive() {
    if (!initialized) {
        return RADIOLIB_ERR_CHIP_NOT_FOUND;
    }

    // Set up ISR callback for reception
    LoRa.onReceive(onReceiveISR);

    // Start continuous RX mode
    LoRa.receive();

    Serial.println("LoRaSemDriver: Started continuous RX mode with interrupt");
    return RADIOLIB_ERR_NONE;
}

bool LoRaSemDriver::available() {
    if (!initialized) {
        return false;
    }

    // Check flag set by ISR
    return packetReceived;
}

int LoRaSemDriver::getRSSI() const {
    return lastRssi;
}

float LoRaSemDriver::getSNR() const {
    return lastSnr;
}

int LoRaSemDriver::getCurrentRSSI() const {
    if (!initialized) {
        return -200;  // Return very low RSSI if not initialized
    }
    // Read instantaneous RSSI on the channel (not packet RSSI)
    return LoRa.rssi();
}

void LoRaSemDriver::setReceiveCallback(void (*callback)(const uint8_t* data, size_t len)) {
    receiveCallback = callback;
}

// Static ISR handler
void LoRaSemDriver::onReceiveISR(int packetSize) {
    if (instance) {
        instance->packetReceived = true;
        instance->lastRssi = LoRa.packetRssi();
        instance->lastSnr = LoRa.packetSnr();
    }
}

void LoRaSemDriver::sleep() {
    if (initialized) {
        LoRa.sleep();
        Serial.println("LoRa: Sleep mode");
    }
}

void LoRaSemDriver::idle() {
    if (initialized) {
        LoRa.idle();
        Serial.println("LoRa: Idle mode");
    }
}
