#ifndef BUNDLE_H
#define BUNDLE_H

#include <Arduino.h>
#include "EndpointID.h"

/**
 * BPv7 Bundle Processing Flags (RFC 9171 Section 4.2.3)
 */
namespace BundleFlags {
    const uint32_t IS_FRAGMENT                   = 0x00000001;
    const uint32_t PAYLOAD_IS_ADMIN_RECORD       = 0x00000002;
    const uint32_t DO_NOT_FRAGMENT               = 0x00000004;
    const uint32_t ACKNOWLEDGEMENT_REQUESTED     = 0x00000020;
    const uint32_t STATUS_TIME_REQUESTED         = 0x00000040;
    const uint32_t RECEPTION_STATUS_REQUESTED    = 0x00004000;
    const uint32_t FORWARDING_STATUS_REQUESTED   = 0x00010000;
    const uint32_t DELIVERY_STATUS_REQUESTED     = 0x00020000;
    const uint32_t DELETION_STATUS_REQUESTED     = 0x00040000;
}

/**
 * BPv7 Primary Block (RFC 9171 Section 4.3.1)
 *
 * Contains bundle metadata and forwarding information.
 */
struct PrimaryBlock {
    uint8_t version;                // BPv7 = 7
    uint32_t bundle_proc_flags;     // Processing flags (see BundleFlags)
    uint32_t crc_type;              // CRC type (0=none, 1=CRC16, 2=CRC32)
    EndpointID destination;         // Destination endpoint
    EndpointID source;              // Source endpoint
    EndpointID report_to;           // Report-to endpoint
    uint64_t creation_timestamp;    // Creation timestamp (ms since 2000-01-01)
    uint32_t sequence_number;       // Sequence number
    uint32_t lifetime_ms;           // Bundle lifetime in milliseconds
    uint32_t fragment_offset;       // Fragment offset (0 if not fragment)
    uint32_t total_data_length;     // Total ADU length (0 if not fragment)

    PrimaryBlock()
        : version(7),
          bundle_proc_flags(0),
          crc_type(0),
          destination(),
          source(),
          report_to(),
          creation_timestamp(0),
          sequence_number(0),
          lifetime_ms(3600000),  // Default: 1 hour
          fragment_offset(0),
          total_data_length(0) {}
};

/**
 * BPv7 Canonical Block Type Codes (RFC 9171 Section 4.3.2)
 */
namespace BlockType {
    const uint8_t PAYLOAD_BLOCK           = 1;
    const uint8_t PREVIOUS_NODE_BLOCK     = 6;
    const uint8_t BUNDLE_AGE_BLOCK        = 7;
    const uint8_t HOP_COUNT_BLOCK         = 10;
}

/**
 * BPv7 Payload Block (RFC 9171 Section 4.3.3)
 *
 * Contains the actual user data.
 */
struct PayloadBlock {
    uint8_t block_type;             // Always BlockType::PAYLOAD_BLOCK (1)
    uint32_t block_number;          // Block number (payload is typically 1)
    uint32_t block_proc_flags;      // Processing flags
    uint32_t crc_type;              // CRC type
    uint8_t* data;                  // Payload data (dynamically allocated)
    size_t data_length;             // Length of payload data

    PayloadBlock()
        : block_type(BlockType::PAYLOAD_BLOCK),
          block_number(1),
          block_proc_flags(0),
          crc_type(0),
          data(nullptr),
          data_length(0) {}

    ~PayloadBlock() {
        if (data != nullptr) {
            delete[] data;
            data = nullptr;
        }
    }

    // Copy constructor
    PayloadBlock(const PayloadBlock& other)
        : block_type(other.block_type),
          block_number(other.block_number),
          block_proc_flags(other.block_proc_flags),
          crc_type(other.crc_type),
          data(nullptr),
          data_length(other.data_length) {
        if (other.data != nullptr && other.data_length > 0) {
            data = new uint8_t[other.data_length];
            memcpy(data, other.data, other.data_length);
        }
    }

    // Assignment operator
    PayloadBlock& operator=(const PayloadBlock& other) {
        if (this != &other) {
            // Free existing data
            if (data != nullptr) {
                delete[] data;
                data = nullptr;
            }

            // Copy fields
            block_type = other.block_type;
            block_number = other.block_number;
            block_proc_flags = other.block_proc_flags;
            crc_type = other.crc_type;
            data_length = other.data_length;

            // Copy data
            if (other.data != nullptr && other.data_length > 0) {
                data = new uint8_t[other.data_length];
                memcpy(data, other.data, other.data_length);
            }
        }
        return *this;
    }

    /**
     * Set payload data (copies the data)
     * @param src Source data pointer
     * @param len Length of data
     */
    void setData(const uint8_t* src, size_t len) {
        // Free existing data
        if (data != nullptr) {
            delete[] data;
            data = nullptr;
        }

        // Allocate and copy new data
        if (src != nullptr && len > 0) {
            data = new uint8_t[len];
            memcpy(data, src, len);
            data_length = len;
        } else {
            data_length = 0;
        }
    }

    /**
     * Set payload data from String
     * @param str String to use as payload
     */
    void setData(const String& str) {
        setData((const uint8_t*)str.c_str(), str.length());
    }
};

/**
 * Bundle Metadata (local use only, not transmitted)
 */
struct BundleMetadata {
    uint32_t local_id;              // Local bundle ID (for tracking)
    uint64_t received_at_ms;        // Local receive timestamp
    uint64_t expires_at_ms;         // Expiration timestamp
    uint8_t hop_count;              // Current hop count
    uint8_t max_hops;               // Maximum hops allowed
    bool locally_generated;         // True if created on this node
    bool forwarded;                 // True if already forwarded
    uint8_t received_via_cla;       // CLA index that received this bundle (0xFF = locally generated)

    BundleMetadata()
        : local_id(0),
          received_at_ms(0),
          expires_at_ms(0),
          hop_count(0),
          max_hops(32),
          locally_generated(false),
          forwarded(false),
          received_via_cla(0xFF) {}
};

/**
 * Complete BPv7 Bundle
 *
 * Represents a complete bundle with primary block, payload, and metadata.
 */
struct Bundle {
    PrimaryBlock primary;
    PayloadBlock payload;
    BundleMetadata metadata;

    Bundle() : primary(), payload(), metadata() {}

    /**
     * Get bundle ID string (source + timestamp + sequence)
     * @return String representation of bundle ID
     */
    String getBundleID() const {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%s-%llu-%lu",
                 primary.source.toString().c_str(),
                 (unsigned long long)primary.creation_timestamp,
                 (unsigned long)primary.sequence_number);
        return String(buffer);
    }

    /**
     * Check if bundle has expired
     * @param current_time_ms Current time in milliseconds
     * @return true if expired, false otherwise
     */
    bool isExpired(uint64_t current_time_ms) const {
        return (current_time_ms >= metadata.expires_at_ms);
    }

    /**
     * Calculate expiration time based on creation + lifetime
     */
    void calculateExpiration() {
        metadata.expires_at_ms = primary.creation_timestamp + primary.lifetime_ms;
    }

    /**
     * Check if bundle is a fragment
     * @return true if IS_FRAGMENT flag is set
     */
    bool isFragment() const {
        return (primary.bundle_proc_flags & BundleFlags::IS_FRAGMENT) != 0;
    }

    /**
     * Check if bundle is an admin record
     * @return true if PAYLOAD_IS_ADMIN_RECORD flag is set
     */
    bool isAdminRecord() const {
        return (primary.bundle_proc_flags & BundleFlags::PAYLOAD_IS_ADMIN_RECORD) != 0;
    }
};

#endif // BUNDLE_H
