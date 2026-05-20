#ifndef LORASEM_DRIVER_H
#define LORASEM_DRIVER_H

#include <LoRaSem.h>

/**
 * LoRaSemDriver - Adapter for LoRaSem (SanddeepMistry LoRa library)
 *
 * Provides a simplified interface compatible with the RadioLibDriver API
 * but using the proven LoRaSem library from LoRaDTN template.
 *
 * RECEPTION MODE - INTERRUPT:
 * ============================
 * This implementation uses INTERRUPT mode:
 * - Uses LoRa.onReceive(callback) for interrupt-based reception
 * - ISR fires on DIO0 pin when packet received
 * - More efficient than polling (CPU can sleep until interrupt)
 * - Reception handled in background automatically
 */
class LoRaSemDriver {
private:
    bool initialized;
    volatile bool packetReceived;
    volatile int lastRssi;
    volatile float lastSnr;

    // Callback for received packets (optional)
    void (*receiveCallback)(const uint8_t* data, size_t len);

    // Static ISR handler
    static void onReceiveISR(int packetSize);
    static LoRaSemDriver* instance;  // Singleton for ISR access

public:
    LoRaSemDriver();
    ~LoRaSemDriver();

    /**
     * Initialize the LoRa radio module
     *
     * @param cs Chip select pin
     * @param dio0 DIO0 pin (interrupt)
     * @param rst Reset pin
     * @param dio1 DIO1 pin (not used by LoRaSem)
     * @param frequency Frequency in MHz (e.g., 434.0 for 434 MHz)
     * @param bandwidth Bandwidth in kHz (e.g., 250.0 for 250 kHz)
     * @param spreadingFactor Spreading factor (6-12)
     * @param codingRate Coding rate denominator (5-8 for 4/5 to 4/8)
     * @param txPower TX power in dBm (2-20 for SX1276)
     * @param syncWord Sync word (0x12 for private networks, 0x34 for LoRaWAN)
     * @param preambleLength Preamble length in symbols (default: 8)
     *
     * @return 0 on success, error code otherwise
     */
    int init(int cs, int dio0, int rst, int dio1,
             float frequency, float bandwidth,
             uint8_t spreadingFactor, uint8_t codingRate,
             int8_t txPower, uint8_t syncWord, uint8_t preambleLength = 8);

    /**
     * Transmit a byte array
     *
     * @param data Pointer to data buffer
     * @param len Length of data
     * @return 0 on success, error code otherwise
     */
    int transmit(const uint8_t* data, size_t len);

    /**
     * Receive data (blocking with timeout)
     *
     * @param buffer Buffer to store received data
     * @param max_len Maximum buffer size
     * @param out_len Actual received length (output)
     * @param timeout_ms Timeout in milliseconds
     * @return 0 on success, error code otherwise
     */
    int receive(uint8_t* buffer, size_t max_len, size_t* out_len, uint32_t timeout_ms = 1000);

    /**
     * Start continuous receive mode
     *
     * @return 0 on success, error code otherwise
     */
    int startReceive();

    /**
     * Check if packet is available (non-blocking)
     *
     * @return true if packet available, false otherwise
     */
    bool available();

    /**
     * Get RSSI of last received packet
     *
     * @return RSSI in dBm
     */
    int getRSSI() const;

    /**
     * Get SNR of last received packet
     *
     * @return SNR in dB
     */
    float getSNR() const;

    /**
     * Get current channel RSSI (not packet RSSI)
     *
     * Used for channel activity detection / listen-before-talk.
     * Returns instantaneous RSSI on the channel.
     *
     * @return Current RSSI in dBm
     */
    int getCurrentRSSI() const;

    /**
     * Set receive callback (optional)
     *
     * @param callback Function pointer to callback
     */
    void setReceiveCallback(void (*callback)(const uint8_t* data, size_t len));

    /**
     * Put radio into sleep mode (lowest power)
     */
    void sleep();

    /**
     * Put radio into idle mode (standby)
     */
    void idle();
};

// Error codes (compatible with RadioLib)
#define RADIOLIB_ERR_NONE 0
#define RADIOLIB_ERR_CHIP_NOT_FOUND -2
#define RADIOLIB_ERR_RX_TIMEOUT -7

#endif // LORASEM_DRIVER_H
