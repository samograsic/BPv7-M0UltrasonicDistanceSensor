#ifndef BUNDLE_MANAGER_H
#define BUNDLE_MANAGER_H

#include <Arduino.h>
#include "Bundle.h"
#include "BPv7Codec.h"

// Default bundle lifetime (can be overridden by config.h)
#ifndef DEFAULT_BUNDLE_LIFETIME_MS
#define DEFAULT_BUNDLE_LIFETIME_MS 1800000  // 30 minutes
#endif

// Forward declaration
class IBundleService;

// Maximum bundles stored in RAM
#define MAX_BUNDLE_STORE_SIZE 32

// Maximum registered services
#define MAX_SERVICES 8

// Maximum CBOR size for LoRa transmission (LoRa frame limit)
#define MAX_LORA_CBOR_SIZE 255

/**
 * Interface for Convergence Layer Adapters
 * Each CLA (LoRa, LTE/MQTT) implements this interface
 */
class IConvergenceLayer {
public:
    virtual ~IConvergenceLayer() {}

    /**
     * Initialize the convergence layer
     * @return true on success
     */
    virtual bool initialize() = 0;

    /**
     * Transmit a bundle via this convergence layer
     * @param cbor_data CBOR-encoded bundle data
     * @param cbor_len Length of CBOR data
     * @return true on success
     */
    virtual bool transmit(const uint8_t* cbor_data, size_t cbor_len) = 0;

    /**
     * Check if a bundle is available (non-blocking)
     * @return true if bundle ready to receive
     */
    virtual bool available() = 0;

    /**
     * Receive a bundle from this convergence layer (non-blocking)
     * @param cbor_buffer Buffer for CBOR-encoded bundle
     * @param max_len Maximum buffer size
     * @param out_len Actual received length
     * @return true on success
     */
    virtual bool receive(uint8_t* cbor_buffer, size_t max_len, size_t* out_len) = 0;

    /**
     * Get convergence layer name for logging
     */
    virtual const char* getName() const = 0;

    /**
     * Get maximum transmission unit (bytes)
     */
    virtual size_t getMTU() const = 0;
};

/**
 * Bundle Storage Interface
 */
class IBundleStore {
public:
    virtual ~IBundleStore() {}

    /**
     * Store a bundle
     * @return true on success
     */
    virtual bool store(const Bundle& bundle) = 0;

    /**
     * Get bundle by index
     * @param index Bundle index (0 to count-1)
     * @param bundle Output bundle
     * @return true if bundle exists
     */
    virtual bool get(size_t index, Bundle& bundle) const = 0;

    /**
     * Remove bundle by index
     * @return true on success
     */
    virtual bool remove(size_t index) = 0;

    /**
     * Get number of bundles stored
     */
    virtual size_t count() const = 0;

    /**
     * Check if bundle already exists (duplicate detection)
     */
    virtual bool exists(const Bundle& bundle) const = 0;

    /**
     * Remove all expired bundles
     * @param current_time_ms Current DTN time in milliseconds
     * @return Number of bundles removed
     */
    virtual size_t removeExpired(uint64_t current_time_ms) = 0;
};

/**
 * Simple RAM-based bundle store
 */
class RamBundleStore : public IBundleStore {
public:
    RamBundleStore();

    bool store(const Bundle& bundle) override;
    bool get(size_t index, Bundle& bundle) const override;
    bool remove(size_t index) override;
    size_t count() const override { return bundle_count; }
    bool exists(const Bundle& bundle) const override;
    size_t removeExpired(uint64_t current_time_ms) override;

private:
    Bundle bundles[MAX_BUNDLE_STORE_SIZE];
    size_t bundle_count;
};

/**
 * Bundle Manager
 *
 * Central management for bundle creation, storage, expiration, and forwarding.
 * Coordinates with Convergence Layer Adapters (CLAs) for transmission/reception.
 */
class BundleManager {
public:
    BundleManager();

    /**
     * Initialize bundle manager
     */
    bool initialize();

    /**
     * Set time source callback for getting current DTN time
     * @param callback Function that returns current DTN time in milliseconds since 2000-01-01
     */
    void setTimeSource(uint64_t (*callback)());

    /**
     * Register a convergence layer adapter
     * @param cla Pointer to CLA instance (must remain valid)
     * @return true on success
     */
    bool registerCLA(IConvergenceLayer* cla);

    /**
     * Register a bundle service
     * @param service Pointer to service instance (must remain valid)
     * @return true on success
     */
    bool registerService(IBundleService* service);

    /**
     * Create and store a new bundle
     * @param destination Destination endpoint
     * @param payload_data Payload data
     * @param payload_len Payload length
     * @param lifetime_ms Bundle lifetime in milliseconds (default: 30 minutes)
     * @return true on success
     */
    bool createBundle(const EndpointID& destination,
                     const uint8_t* payload_data,
                     size_t payload_len,
                     uint32_t lifetime_ms = DEFAULT_BUNDLE_LIFETIME_MS);

    /**
     * Create a new bundle with custom source service number
     * @param destination Destination endpoint ID
     * @param payload_data Payload data buffer
     * @param payload_len Payload data length
     * @param lifetime_ms Bundle lifetime in milliseconds
     * @param source_service_number Custom service number for source EID
     * @return true if bundle created successfully
     */
    bool createBundle(const EndpointID& destination,
                     const uint8_t* payload_data,
                     size_t payload_len,
                     uint32_t lifetime_ms,
                     uint32_t source_service_number);

    /**
     * Process a received CBOR-encoded bundle
     * @param cbor_data CBOR bundle data
     * @param cbor_len CBOR data length
     * @param rssi Received signal strength (optional, 0 if N/A)
     * @param snr Signal-to-noise ratio (optional, 0 if N/A)
     * @param cla_name Name of the CLA that received this bundle (nullptr for locally generated)
     * @return true if bundle processed successfully
     */
    bool processReceivedBundle(const uint8_t* cbor_data,
                              size_t cbor_len,
                              int16_t rssi = 0,
                              float snr = 0.0f,
                              const char* cla_name = nullptr);

    /**
     * Perform epidemic forwarding - send all unsent bundles via all CLAs
     * This should be called periodically (e.g., in SERVICE_LORA_CLA state)
     * @return Number of bundles transmitted
     */
    uint16_t forwardBundles();

    /**
     * Forward bundles via specific CLA only
     * @param cla_name Name of CLA to use (e.g., "LoRa", "LTE")
     * @return Number of bundles transmitted
     */
    uint16_t forwardBundlesViaCLA(const char* cla_name);

    /**
     * Remove expired bundles from store
     * @return Number of bundles removed
     */
    uint16_t removeExpiredBundles();

    /**
     * Remove bundles that have been fully forwarded via all CLAs
     * (Epidemic forwarding: sent via both LoRa and LTE, or just LoRa if received via LTE)
     * @return Number of bundles removed
     */
    uint16_t removeForwardedBundles();

    /**
     * Mark all bundles added at or after from_store_index as already sent via cla_name.
     * Use this immediately after createBundle() to prevent forwarding via a specific CLA.
     * @param from_store_index getBundleCount() snapshot taken before the createBundle() calls
     * @param cla_name         Name of the CLA to mark as already sent (e.g. "LoRa")
     */
    void markBundlesSentViaCLA(size_t from_store_index, const char* cla_name);

    /**
     * Get bundle count
     */
    size_t getBundleCount() const { return store.count(); }

    /**
     * Set local endpoint ID (node number)
     */
    void setLocalEID(uint32_t node_number, uint32_t service_number = 0);

    /**
     * Get local endpoint ID
     */
    EndpointID getLocalEID() const { return local_eid; }

    /**
     * Get current DTN time (milliseconds since 2000-01-01)
     */
    uint64_t getCurrentDtnTime() const;

    /**
     * Get next sequence number for bundle creation
     */
    uint64_t getNextSequenceNumber();

private:
    // Bundle storage
    RamBundleStore store;

    // Local endpoint ID
    EndpointID local_eid;

    // Sequence number counter
    uint64_t sequence_counter;

    // Time source callback (for getting DTN time)
    uint64_t (*time_source_callback)();

    // Registered CLAs (max 4: LoRa, LTE, future extensions)
    IConvergenceLayer* clas[4];
    uint8_t cla_count;

    // Registered services
    IBundleService* services[MAX_SERVICES];
    uint8_t service_count;

    // Tracking which bundles sent via which CLA
    struct BundleTransmitRecord {
        uint32_t bundle_local_id;
        uint8_t sent_via_cla_mask;  // Bit mask: bit 0=CLA0, bit 1=CLA1, etc.
    };
    BundleTransmitRecord tx_records[MAX_BUNDLE_STORE_SIZE];
    uint8_t tx_record_count;

    /**
     * Check if bundle already sent via specific CLA
     */
    bool isSentViaCLA(uint32_t bundle_local_id, uint8_t cla_index);

    /**
     * Mark bundle as sent via specific CLA
     */
    void markSentViaCLA(uint32_t bundle_local_id, uint8_t cla_index);

    /**
     * Get or create transmit record for bundle
     */
    BundleTransmitRecord* getTxRecord(uint32_t bundle_local_id);

    /**
     * Remove TX record for a bundle (called when bundle is removed from store)
     */
    void removeTxRecord(uint32_t bundle_local_id);

    /**
     * Deliver bundle to local service
     * @param bundle Bundle to deliver
     * @return service name if delivered, nullptr if no matching service
     */
    const char* deliverToLocalService(const Bundle& bundle);
};

#endif // BUNDLE_MANAGER_H
