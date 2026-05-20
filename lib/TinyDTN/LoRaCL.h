#ifndef LORA_CL_H
#define LORA_CL_H

#include "BundleManager.h"
#include "../../lib/lorasem_adapter/LoRaSemDriver.h"

// LoRa neighbor entry (source EID node + last-seen timestamp + radio stats)
struct LoRaNeighbor {
    uint32_t node_number;
    uint32_t last_seen_ms;      // millis() when last heard
    int16_t  rssi;
    float    snr;
    float    tx_duty_cycle;     // TX duty cycle reported by neighbor via LoRa extension block (%)
};

/**
 * LoRa Convergence Layer Adapter
 *
 * Implements IConvergenceLayer interface for LoRa radio.
 * Transmits and receives pure BPv7 CBOR bundles over LoRa radio.
 *
 * No additional framing or headers - just raw CBOR bundle data.
 *
 * Broadcast Bundle Handling:
 * - Broadcast bundles (destination ipn:0.0) are generated and processed ONLY in this CLA layer
 * - Broadcast bundles NEVER touch the Bundle Manager layer
 * - Broadcast beacon includes: transmitting EID, RTC timestamp, neighbor list (future)
 *
 * LoRa Extension Block:
 * - Added when transmitting bundles via LoRa
 * - Removed when receiving bundles (processed by CLA)
 * - Contains: transmitter EID, ACKs, etc.
 * - Used to fill bundles <255 bytes to improve unreliable radio link
 */
class LoRaCL : public IConvergenceLayer {
public:
    LoRaCL(int cs_pin, int dio0_pin, int rst_pin,
           float freq_mhz, float bw_khz,
           uint8_t sf, uint8_t cr, int8_t tx_power, uint8_t sync_word, uint8_t preamble_len = 8);

    // IConvergenceLayer interface
    bool initialize() override;
    bool transmit(const uint8_t* cbor_data, size_t cbor_len) override;
    bool available() override;
    bool receive(uint8_t* cbor_buffer, size_t max_len, size_t* out_len) override;
    const char* getName() const override { return "LoRa"; }
    size_t getMTU() const override { return 255; }  // LoRa frame limit

    // Additional LoRa-specific methods
    int16_t getLastRSSI() const;
    float getLastSNR() const;

    // Neighbor table (source EIDs of nodes heard over LoRa)
    uint8_t getNeighborCount() const { return neighbor_count; }
    const LoRaNeighbor* getNeighbors() const { return neighbors; }

    // TX Duty Cycle Tracking
    // Returns TX duty cycle percentage for the last hour (0.0 - 100.0)
    float getTxDutyCycle() const;
    // Returns total TX time in milliseconds for the last hour
    uint32_t getTxTimeLastHour() const;

    // LoRa Extension Block type code (private/experimental range per RFC 9171)
    static const uint8_t LORA_EXT_BLOCK_TYPE = 192;  // 0xC0

    // Power management
    void sleep();         // Put radio into sleep mode (lowest power)
    void idle();          // Put radio into idle mode (standby)
    void startReceive();  // Wake radio from sleep/idle back into continuous RX

    // Broadcast Bundle Handling (CLA-layer only, never touches Bundle Manager)
    // Generates and transmits a broadcast beacon bundle (destination ipn:0.0)
    // Beacon contains: transmitting EID, RTC timestamp, neighbor list (future)
    bool transmitBroadcastBeacon(const EndpointID& source_eid, uint64_t rtc_timestamp);

    // Process received broadcast bundle (CLA-layer only)
    // Returns true if bundle was a broadcast and was processed
    bool processBroadcastBundle(const uint8_t* cbor_data, size_t cbor_len, int16_t rssi, float snr);

private:
    LoRaSemDriver radio;

    // Radio configuration
    int cs_pin;
    int dio0_pin;
    int rst_pin;
    float freq_mhz;
    float bw_khz;
    uint8_t sf;
    uint8_t cr;
    int8_t tx_power;
    uint8_t sync_word;
    uint8_t preamble_len;

    bool initialized;

    // TX Duty Cycle Tracking (last hour)
    static const uint8_t MAX_TX_RECORDS = 100;  // Maximum TX events to track
    struct TxRecord {
        uint32_t start_time_ms;   // millis() when TX started
        uint32_t duration_ms;     // TX duration in milliseconds
    };
    TxRecord tx_records[MAX_TX_RECORDS];
    uint8_t tx_record_count;
    uint8_t tx_record_index;  // Rolling index for circular buffer

    // Helper methods for duty cycle tracking
    void recordTxEvent(uint32_t duration_ms);
    void cleanOldTxRecords();  // Remove records older than 1 hour

    // Collision avoidance helpers
    bool isChannelBusy();  // Check if channel is busy using RSSI threshold
    int transmitWithCollisionAvoidance(const uint8_t* data, size_t len);  // Transmit with LBT

    // Neighbor table (tracks source EIDs from received bundles)
    static const uint8_t MAX_LORA_NEIGHBORS = 30;
    LoRaNeighbor neighbors[MAX_LORA_NEIGHBORS];
    uint8_t neighbor_count;

    bool updateNeighbor(uint32_t node_number, int16_t rssi, float snr, float duty_cycle = 0.0f); // returns true if new neighbor
    void cleanExpiredNeighbors();

    // LoRa Extension Block handling
    // Searches for and strips the LoRa extension block from a received bundle.
    // If found, extracts transmitter node and duty cycle; returns true.
    bool processLoRaExtensionBlock(uint8_t* cbor_data, size_t* cbor_len,
                                   uint32_t* out_node, float* out_duty_cycle);

    // Injects a LoRa extension block into cbor_data before the final 0xFF break.
    // Encodes: [transmitter_node_number, fragment_flag=0, tx_duty_cycle].
    // Returns false (no-op) if the bundle would exceed max_len after injection.
    bool addLoRaExtensionBlock(uint8_t* cbor_data, size_t* cbor_len, size_t max_len,
                               const EndpointID& transmitter_eid);
};

// Global LoRa CL instance (defined in main.cpp)
extern LoRaCL loRaCL;

#endif // LORA_CL_H
