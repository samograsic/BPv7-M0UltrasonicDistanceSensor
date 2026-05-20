#include "BPv7Codec.h"

// Debug output control (set to 1 to enable verbose logging)
#define BPV7_DEBUG 0

// CBOR major types
#define CBOR_UINT 0x00
#define CBOR_BYTES 0x40
#define CBOR_ARRAY 0x80

// Debug macros
#if BPV7_DEBUG
    #define BPV7_DEBUG_PRINT(x) Serial.print(x)
    #define BPV7_DEBUG_PRINTLN(x) Serial.println(x)
    #define BPV7_DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
    #define BPV7_DEBUG_PRINT(x)
    #define BPV7_DEBUG_PRINTLN(x)
    #define BPV7_DEBUG_PRINTF(...)
#endif

bool BPv7Codec::encode(const Bundle& bundle, uint8_t* output, size_t max_len, size_t* out_len) {
    // RFC 9171: Bundle is an indefinite-length CBOR array
    // Format: 0x9F <primary block> <extension blocks...> <payload block> 0xFF

    if (output == nullptr || out_len == nullptr || max_len == 0) {
        return false;
    }

    size_t offset = 0;

    // Start indefinite-length array (0x9F)
    if (max_len < 1) return false;
    output[offset++] = 0x9F;

    // Encode primary block
    size_t primary_len = 0;
    if (!encodePrimaryBlock(bundle.primary, output + offset, max_len - offset, &primary_len)) {
        return false;
    }
    offset += primary_len;

    // Encode payload block
    size_t payload_len = 0;
    if (!encodePayloadBlock(bundle.payload, output + offset, max_len - offset, &payload_len)) {
        return false;
    }
    offset += payload_len;

    // End indefinite-length array (0xFF - CBOR break)
    if (offset >= max_len) return false;
    output[offset++] = 0xFF;

    *out_len = offset;
    return true;
}

bool BPv7Codec::decode(const uint8_t* input, size_t input_len, Bundle& bundle) {
    if (input == nullptr || input_len == 0) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: decode() - input is null or empty");
        return false;
    }

    // Check CBOR format
    uint8_t firstByte = input[0];
    uint8_t majorType = (firstByte >> 5) & 0x07;
    uint8_t addInfo = firstByte & 0x1F;

    BPV7_DEBUG_PRINTF("BPv7Codec: First byte 0x%02X - Major type: %d, Additional info: %d\n",
                  firstByte, majorType, addInfo);

    size_t offset = 0;

    // LoRaDTN custom format: 0xA5 + 0x9F (bundle wrapper) + blocks
    // Standard BPv7: Array (0x9F or 0x8x) containing blocks
    if (firstByte == 0xA5) {
        // LoRaDTN format - 0xA5 header + 0x9F wrapper + blocks
        BPV7_DEBUG_PRINTLN("BPv7Codec: Detected LoRaDTN format (0xA5 header)");
        offset = 1;  // Skip 0xA5 header

        // Check for 0x9F bundle wrapper (indefinite array)
        if (input[offset] == 0x9F) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: Skipping 0x9F bundle wrapper");
            offset++;  // Skip 0x9F wrapper
        }

        // Decode primary block (first block after header and wrapper)
        size_t consumed = 0;
        if (!decodePrimaryBlock(input + offset, input_len - offset, bundle.primary, &consumed)) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: ERROR - Failed to decode primary block");
            return false;
        }
        offset += consumed;

        // Now we need to find the payload block
        // Skip intermediate blocks until we find payload
        bool found_payload = false;
        int block_attempts = 0;
        while (offset < input_len && !found_payload && block_attempts < 100) {
            block_attempts++;
            uint8_t blockHeader = input[offset];
            uint8_t blockMajor = (blockHeader >> 5) & 0x07;
            uint8_t blockInfo = blockHeader & 0x1F;

            BPV7_DEBUG_PRINTF("BPv7Codec: Checking block at offset %d: 0x%02X (major:%d, info:%d)\n",
                         offset, blockHeader, blockMajor, blockInfo);

            // Check for break marker (0xFF) - end of indefinite array
            if (blockHeader == 0xFF) {
                BPV7_DEBUG_PRINTLN("BPv7Codec: Found CBOR break marker (0xFF), stopping block search");
                break;
            }

            // Payload block format: array with elements
            // Try to decode as payload if it's an array
            if (blockMajor == 4) {  // CBOR array
                size_t block_start = offset;
                if (decodePayloadBlock(input + offset, input_len - offset, bundle.payload, &consumed)) {
                    BPV7_DEBUG_PRINTLN("BPv7Codec: Successfully decoded payload block");
                    offset += consumed;
                    found_payload = true;
                    break;
                } else {
                    BPV7_DEBUG_PRINTLN("BPv7Codec: Block is array but not payload block (type != 1), skipping");
                    // Reset offset to block start and skip by 1 byte
                    offset = block_start + 1;
                }
            } else {
                // Not an array, skip this byte
                offset++;
            }
        }

        if (!found_payload) {
            BPV7_DEBUG_PRINTF("BPv7Codec: ERROR - Payload block not found after %d attempts (last offset: %d)\n",
                         block_attempts, offset);
            return false;
        }
    } else if (majorType == 4 || firstByte == 0x9F) {
        // Array format - legacy or alternative encoding
        BPV7_DEBUG_PRINTLN("BPv7Codec: Detected array-based bundle format");
        offset = 1;  // Skip array header

        // Decode primary block
        size_t consumed = 0;
        if (!decodePrimaryBlock(input + offset, input_len - offset, bundle.primary, &consumed)) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: ERROR - Failed to decode primary block");
            return false;
        }
        offset += consumed;

        // Search for payload block, skipping extension blocks
        bool found_payload = false;
        int block_attempts = 0;
        while (offset < input_len && !found_payload && block_attempts < 100) {
            block_attempts++;
            uint8_t blockHeader = input[offset];
            uint8_t blockMajor = (blockHeader >> 5) & 0x07;

            BPV7_DEBUG_PRINTF("BPv7Codec: Checking block at offset %d: 0x%02X (major:%d)\n",
                         offset, blockHeader, blockMajor);

            // Check for break marker (0xFF) - end of indefinite array
            if (blockHeader == 0xFF) {
                BPV7_DEBUG_PRINTLN("BPv7Codec: Found CBOR break marker (0xFF), stopping block search");
                break;
            }

            // Try to decode as payload block if it's an array
            if (blockMajor == 4) {  // CBOR array
                size_t block_start = offset;
                if (decodePayloadBlock(input + offset, input_len - offset, bundle.payload, &consumed)) {
                    BPV7_DEBUG_PRINTLN("BPv7Codec: Successfully decoded payload block");
                    offset += consumed;
                    found_payload = true;
                    break;
                } else {
                    BPV7_DEBUG_PRINTLN("BPv7Codec: Block is array but not payload block, skipping");
                    // Reset offset to block start and skip by 1 byte
                    offset = block_start + 1;
                }
            } else {
                // Not an array, skip this byte
                offset++;
            }
        }

        if (!found_payload) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: ERROR - Failed to find payload block");
            return false;
        }
    } else {
        BPV7_DEBUG_PRINTF("BPv7Codec: ERROR - Expected CBOR array or map, got major type %d\n", majorType);
        return false;
    }

    BPV7_DEBUG_PRINTLN("BPv7Codec: Bundle decoded successfully!");
    return true;
}

size_t BPv7Codec::estimateSize(const Bundle& bundle) {
    // Rough estimate: primary block (~100 bytes) + payload data + overhead
    return 100 + bundle.payload.data_length + 50;
}

bool BPv7Codec::encodePrimaryBlock(const PrimaryBlock& primary, uint8_t* output, size_t max_len, size_t* out_len) {
    // Simplified encoding for Phase 1
    // Real CBOR encoding: indefinite-length array with version, flags, CRC type, dest, source, etc.

    if (max_len < 100) {  // Minimum space needed
        return false;
    }

    size_t offset = 0;

    // Reference implementation uses 9-element primary block with CRC
    // [version, flags, crc_type, destination, source, report_to, creation_timestamp, lifetime, crc_value]
    // NOTE: We encode with CRC type 1 (CRC16) to match ION expectations

    // Encode as CBOR array of 9 elements (to match working reference)
    if (!encodeCBORArrayHeader(9, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Version
    if (!encodeCBORUint(primary.version, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Flags
    if (!encodeCBORUint(primary.bundle_proc_flags, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // CRC type
    if (!encodeCBORUint(primary.crc_type, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Destination EID
    if (!encodeEndpointID(primary.destination, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Source EID
    if (!encodeEndpointID(primary.source, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Report-to EID
    if (!encodeEndpointID(primary.report_to, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Creation timestamp (RFC 9171: 2-element array [timestamp, sequence_number])
    if (!encodeCBORArrayHeader(2, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Timestamp value
    if (!encodeCBORUint(primary.creation_timestamp, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Sequence number value
    if (!encodeCBORUint(primary.sequence_number, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Lifetime
    if (!encodeCBORUint(primary.lifetime_ms, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Add placeholder CRC field (2-byte CBOR byte string with zeros)
    // Reference: CRC is calculated over the entire block WITH the CRC field set to zeros
    uint8_t zero_crc[2] = {0, 0};
    size_t crc_field_start = offset;  // Remember where CRC field starts
    if (!encodeCBORBytes(zero_crc, 2, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Now calculate CRC16 over the entire primary block (including the zero CRC field)
    uint16_t crc = calculateCRC16(output, offset);

    // Replace the zero CRC bytes with the actual calculated CRC
    // The CBOR byte string format is: 0x42 <byte1> <byte2>
    // So we write the CRC at crc_field_start + 1 (skip the 0x42 header)
    output[crc_field_start + 1] = (uint8_t)(crc >> 8);   // High byte
    output[crc_field_start + 2] = (uint8_t)(crc & 0xFF); // Low byte

    *out_len = offset;
    return true;
}

bool BPv7Codec::encodePayloadBlock(const PayloadBlock& payload, uint8_t* output, size_t max_len, size_t* out_len) {
    // Reference: Payload block is a definite-length CBOR array of 5 elements
    // [block_type, block_number, flags, crc_type, data]

    size_t offset = 0;

    // Encode as CBOR array of 5 elements
    if (!encodeCBORArrayHeader(5, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Block type
    if (!encodeCBORUint(payload.block_type, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Block number
    if (!encodeCBORUint(payload.block_number, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Flags
    if (!encodeCBORUint(payload.block_proc_flags, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // CRC type
    if (!encodeCBORUint(payload.crc_type, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Payload data (as CBOR byte string)
    if (!encodeCBORBytes(payload.data, payload.data_length, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    *out_len = offset;
    return true;
}

bool BPv7Codec::decodePrimaryBlock(const uint8_t* input, size_t input_len, PrimaryBlock& primary, size_t* consumed) {
    if (input == nullptr || input_len < 10) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: decodePrimaryBlock - insufficient data");
        return false;
    }

    size_t offset = 0;
    size_t read = 0;
    uint64_t value = 0;

    // Primary block is an array: [version, flags, crc_type, dest, source, report_to, timestamp, sequence, lifetime]

    // Check for array header
    if (input[offset] == 0x89) {
        // Array of 9 elements
        offset++;
    } else if (input[offset] == 0x9F) {
        // Indefinite array
        offset++;
    } else {
        BPV7_DEBUG_PRINTF("BPv7Codec: Expected primary block array, got 0x%02X\n", input[offset]);
        return false;
    }

    // Version
    if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode version");
        return false;
    }
    primary.version = (uint8_t)value;
    offset += read;
    // Serial.printf("  Version: %d\n", primary.version);

    // Flags
    if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode flags");
        return false;
    }
    primary.bundle_proc_flags = (uint32_t)value;
    offset += read;
    // Serial.printf("  Flags: 0x%04X\n", primary.bundle_proc_flags);

    // CRC type
    if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode CRC type");
        return false;
    }
    primary.crc_type = (uint8_t)value;
    offset += read;

    // Destination EID
    if (!decodeEndpointID(input + offset, input_len - offset, primary.destination, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode destination EID");
        return false;
    }
    offset += read;
    // Serial.printf("  Destination: ipn:%lu.%lu\n", primary.destination.node_number, primary.destination.service_number);

    // Source EID
    if (!decodeEndpointID(input + offset, input_len - offset, primary.source, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode source EID");
        return false;
    }
    offset += read;
    // Serial.printf("  Source: ipn:%lu.%lu\n", primary.source.node_number, primary.source.service_number);

    // Report-to EID
    if (!decodeEndpointID(input + offset, input_len - offset, primary.report_to, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode report-to EID");
        return false;
    }
    offset += read;

    // Creation timestamp - RFC 9171: this is [dtn_time, sequence_number] array
    // BPV7_DEBUG_PRINTF("BPv7Codec: Decoding timestamp at offset %d, byte: 0x%02X\n", offset, input[offset]);

    if (input[offset] == 0x82) {
        // Standard RFC 9171 format: [dtn_time, sequence]
        // BPV7_DEBUG_PRINTLN("BPv7Codec: Creation timestamp is 2-element array [dtn_time, sequence]");
        offset++;  // Skip array header

        // Decode DTN time
        if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode DTN time");
            return false;
        }
        primary.creation_timestamp = value;
        offset += read;
        // BPV7_DEBUG_PRINTF("BPv7Codec: DTN time: %llu\n", primary.creation_timestamp);

        // Decode sequence number (part of creation timestamp in RFC 9171)
        if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode sequence number");
            return false;
        }
        primary.sequence_number = (uint32_t)value;
        offset += read;
        // BPV7_DEBUG_PRINTF("BPv7Codec: Sequence number: %lu\n", primary.sequence_number);
    } else {
        // Legacy format: timestamp and sequence as separate fields
        // BPV7_DEBUG_PRINTLN("BPv7Codec: Legacy timestamp format (separate fields)");

        if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode timestamp");
            return false;
        }
        primary.creation_timestamp = value;
        offset += read;
        // BPV7_DEBUG_PRINTF("BPv7Codec: Timestamp: %llu\n", primary.creation_timestamp);

        // Sequence number (separate field in legacy format)
        if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode sequence");
            return false;
        }
        primary.sequence_number = (uint32_t)value;
        offset += read;
        // BPV7_DEBUG_PRINTF("BPv7Codec: Sequence number: %lu\n", primary.sequence_number);
    }

    // Lifetime
    if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode lifetime");
        return false;
    }
    primary.lifetime_ms = value;
    offset += read;

    // Skip CRC if present (indicated by crc_type > 0)
    if (primary.crc_type > 0) {
        // CRC is a byte string, skip it
        if (input[offset] >= 0x40 && input[offset] <= 0x5F) {
            uint8_t crc_len = input[offset] & 0x1F;
            offset += 1 + crc_len;
        }
    }

    *consumed = offset;
    // BPV7_DEBUG_PRINTF("BPv7Codec: Primary block decoded (%d bytes)\n", offset);
    return true;
}

bool BPv7Codec::decodePayloadBlock(const uint8_t* input, size_t input_len, PayloadBlock& payload, size_t* consumed) {
    if (input == nullptr || input_len < 5) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: decodePayloadBlock - insufficient data");
        return false;
    }

    size_t offset = 0;
    size_t read = 0;
    uint64_t value = 0;

    // Payload block is an array: [block_type, block_number, block_flags, crc_type, data]

    // Check for array header
    if (input[offset] == 0x85) {
        // Array of 5 elements
        offset++;
    } else if (input[offset] == 0x9F) {
        // Indefinite array
        offset++;
    } else {
        BPV7_DEBUG_PRINTF("BPv7Codec: Expected payload block array, got 0x%02X\n", input[offset]);
        return false;
    }

    // Block type (should be 1 for payload)
    if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode block type");
        return false;
    }
    payload.block_type = (uint8_t)value;
    offset += read;
    // Serial.printf("  Block type: %d\n", payload.block_type);

    // Validate this is actually a payload block (type 1)
    if (payload.block_type != 1) {
        BPV7_DEBUG_PRINTF("BPv7Codec: Not a payload block (type=%d), skipping\n", payload.block_type);
        return false;
    }

    // Block number
    if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode block number");
        return false;
    }
    payload.block_number = (uint8_t)value;
    offset += read;

    // Block processing flags
    if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode block flags");
        return false;
    }
    payload.block_proc_flags = (uint32_t)value;
    offset += read;

    // CRC type
    if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode CRC type");
        return false;
    }
    payload.crc_type = (uint8_t)value;
    offset += read;

    // Data (byte string)
    uint8_t* data_ptr = nullptr;
    size_t data_len = 0;
    if (!decodeCBORBytes(input + offset, input_len - offset, &data_ptr, &data_len, &read)) {
        BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode payload data");
        return false;
    }
    offset += read;

    // Copy data to payload
    if (data_len > 0 && data_len <= 255) {
        payload.setData(data_ptr, data_len);
        // Serial.printf("  Payload data: %d bytes\n", data_len);
    } else {
        BPV7_DEBUG_PRINTF("BPv7Codec: Invalid payload data length: %d\n", data_len);
        return false;
    }

    // Skip CRC if present
    if (payload.crc_type > 0) {
        if (offset < input_len && input[offset] >= 0x40 && input[offset] <= 0x5F) {
            uint8_t crc_len = input[offset] & 0x1F;
            offset += 1 + crc_len;
        }
    }

    *consumed = offset;
    // BPV7_DEBUG_PRINTF("BPv7Codec: Payload block decoded (%d bytes)\n", offset);
    return true;
}

bool BPv7Codec::encodeEndpointID(const EndpointID& eid, uint8_t* output, size_t max_len, size_t* out_len) {
    // IPN EID: CBOR array [scheme, [node_number, service_number]]
    // RFC 9171: Two-element array with scheme code and nested SSP array
    // Where scheme=2 is the IPN scheme identifier

    size_t offset = 0;

    // Outer array of 2 elements: [scheme, [node, service]]
    if (!encodeCBORArrayHeader(2, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Scheme (2 = IPN)
    if (!encodeCBORUint(2, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Inner array of 2 elements: [node, service]
    if (!encodeCBORArrayHeader(2, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Node number
    if (!encodeCBORUint(eid.node_number, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    // Service number
    if (!encodeCBORUint(eid.service_number, output + offset, max_len - offset, out_len)) {
        return false;
    }
    offset += *out_len;

    *out_len = offset;
    return true;
}

bool BPv7Codec::decodeEndpointID(const uint8_t* input, size_t input_len, EndpointID& eid, size_t* consumed) {
    if (input == nullptr || input_len < 3) {
        return false;
    }

    size_t offset = 0;
    size_t read = 0;
    uint64_t value = 0;

    // EID is array: [scheme, node, service]
    // Check for array of 2 or 3 elements
    // BPV7_DEBUG_PRINTF("BPv7Codec: decodeEndpointID - first byte: 0x%02X\n", input[offset]);

    if (input[offset] == 0x82) {
        // Array of 2 elements - could be [node, service] OR [scheme, [node,service]]
        // BPV7_DEBUG_PRINTLN("BPv7Codec: EID is 2-element array");
        offset++;

        // Peek at first element
        // BPV7_DEBUG_PRINTF("BPv7Codec: First element byte: 0x%02X\n", input[offset]);
        if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
            BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode first element");
            return false;
        }

        // Check if this is nested format: [scheme, [node, service]]
        if (value <= 3 && offset + read < input_len && input[offset + read] == 0x82) {
            // Nested format detected
            // BPV7_DEBUG_PRINTF("BPv7Codec: Nested EID format - scheme=%llu\n", value);
            offset += read;
            offset++;  // Skip nested array header (0x82)

            // Node number
            // BPV7_DEBUG_PRINTF("BPv7Codec: Decoding node at offset %d, byte: 0x%02X\n", offset, input[offset]);
            if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
                BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode node");
                return false;
            }
            eid.node_number = (uint32_t)value;
            // BPV7_DEBUG_PRINTF("BPv7Codec: Node: %lu\n", eid.node_number);
            offset += read;

            // Service number
            // BPV7_DEBUG_PRINTF("BPv7Codec: Decoding service at offset %d, byte: 0x%02X\n", offset, input[offset]);
            if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
                BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode service");
                return false;
            }
            eid.service_number = (uint32_t)value;
            // BPV7_DEBUG_PRINTF("BPv7Codec: Service: %lu\n", eid.service_number);
            offset += read;
        } else {
            // Standard [node, service] format
            // BPV7_DEBUG_PRINTLN("BPv7Codec: Standard [node, service] format");
            eid.node_number = (uint32_t)value;
            // BPV7_DEBUG_PRINTF("BPv7Codec: Node: %lu\n", eid.node_number);
            offset += read;

            // Service number
            // BPV7_DEBUG_PRINTF("BPv7Codec: Decoding service at offset %d, byte: 0x%02X\n", offset, input[offset]);
            if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
                BPV7_DEBUG_PRINTLN("BPv7Codec: Failed to decode service");
                return false;
            }
            eid.service_number = (uint32_t)value;
            // BPV7_DEBUG_PRINTF("BPv7Codec: Service: %lu\n", eid.service_number);
            offset += read;
        }

    } else if (input[offset] == 0x83) {
        // Array of 3 elements [scheme, node, service]
        offset++;

        // Scheme (should be 2 for IPN)
        if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
            return false;
        }
        offset += read;

        // Node number
        if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
            return false;
        }
        eid.node_number = (uint32_t)value;
        offset += read;

        // Service number
        if (!decodeCBORUint(input + offset, input_len - offset, &value, &read)) {
            return false;
        }
        eid.service_number = (uint32_t)value;
        offset += read;
    } else {
        BPV7_DEBUG_PRINTF("BPv7Codec: Expected EID array, got 0x%02X\n", input[offset]);
        return false;
    }

    *consumed = offset;
    return true;
}

bool BPv7Codec::encodeCBORUint(uint64_t value, uint8_t* output, size_t max_len, size_t* out_len) {
    if (output == nullptr || max_len == 0) {
        return false;
    }

    // CBOR unsigned integer encoding
    if (value < 24) {
        // Tiny value: encode in type byte
        if (max_len < 1) return false;
        output[0] = CBOR_UINT | (uint8_t)value;
        *out_len = 1;
    } else if (value <= 0xFF) {
        // 1-byte value
        if (max_len < 2) return false;
        output[0] = CBOR_UINT | 24;
        output[1] = (uint8_t)value;
        *out_len = 2;
    } else if (value <= 0xFFFF) {
        // 2-byte value
        if (max_len < 3) return false;
        output[0] = CBOR_UINT | 25;
        output[1] = (uint8_t)(value >> 8);
        output[2] = (uint8_t)value;
        *out_len = 3;
    } else if (value <= 0xFFFFFFFF) {
        // 4-byte value
        if (max_len < 5) return false;
        output[0] = CBOR_UINT | 26;
        output[1] = (uint8_t)(value >> 24);
        output[2] = (uint8_t)(value >> 16);
        output[3] = (uint8_t)(value >> 8);
        output[4] = (uint8_t)value;
        *out_len = 5;
    } else {
        // 8-byte value
        if (max_len < 9) return false;
        output[0] = CBOR_UINT | 27;
        output[1] = (uint8_t)(value >> 56);
        output[2] = (uint8_t)(value >> 48);
        output[3] = (uint8_t)(value >> 40);
        output[4] = (uint8_t)(value >> 32);
        output[5] = (uint8_t)(value >> 24);
        output[6] = (uint8_t)(value >> 16);
        output[7] = (uint8_t)(value >> 8);
        output[8] = (uint8_t)value;
        *out_len = 9;
    }

    return true;
}

bool BPv7Codec::decodeCBORUint(const uint8_t* input, size_t input_len, uint64_t* value, size_t* consumed) {
    if (input == nullptr || input_len == 0) {
        return false;
    }

    uint8_t majorType = (input[0] >> 5) & 0x07;
    uint8_t addInfo = input[0] & 0x1F;

    // Must be unsigned int (major type 0)
    if (majorType != 0) {
        return false;
    }

    if (addInfo < 24) {
        // Value 0-23 encoded in additional info
        *value = addInfo;
        *consumed = 1;
        return true;
    } else if (addInfo == 24) {
        // 1-byte uint
        if (input_len < 2) return false;
        *value = input[1];
        *consumed = 2;
        return true;
    } else if (addInfo == 25) {
        // 2-byte uint
        if (input_len < 3) return false;
        *value = ((uint16_t)input[1] << 8) | input[2];
        *consumed = 3;
        return true;
    } else if (addInfo == 26) {
        // 4-byte uint
        if (input_len < 5) return false;
        *value = ((uint32_t)input[1] << 24) | ((uint32_t)input[2] << 16) |
                 ((uint32_t)input[3] << 8) | input[4];
        *consumed = 5;
        return true;
    } else if (addInfo == 27) {
        // 8-byte uint
        if (input_len < 9) return false;
        *value = ((uint64_t)input[1] << 56) | ((uint64_t)input[2] << 48) |
                 ((uint64_t)input[3] << 40) | ((uint64_t)input[4] << 32) |
                 ((uint64_t)input[5] << 24) | ((uint64_t)input[6] << 16) |
                 ((uint64_t)input[7] << 8) | input[8];
        *consumed = 9;
        return true;
    }

    return false;
}

bool BPv7Codec::encodeCBORBytes(const uint8_t* data, size_t data_len, uint8_t* output, size_t max_len, size_t* out_len) {
    if (output == nullptr || max_len == 0) {
        return false;
    }

    size_t offset = 0;

    // Encode byte string length
    size_t len_encoded = 0;
    if (data_len < 24) {
        if (max_len < 1) return false;
        output[0] = CBOR_BYTES | (uint8_t)data_len;
        len_encoded = 1;
    } else if (data_len <= 0xFF) {
        if (max_len < 2) return false;
        output[0] = CBOR_BYTES | 24;
        output[1] = (uint8_t)data_len;
        len_encoded = 2;
    } else if (data_len <= 0xFFFF) {
        if (max_len < 3) return false;
        output[0] = CBOR_BYTES | 25;
        output[1] = (uint8_t)(data_len >> 8);
        output[2] = (uint8_t)data_len;
        len_encoded = 3;
    } else {
        // For embedded systems, we limit to 64KB byte strings
        return false;
    }
    offset += len_encoded;

    // Copy data
    if (max_len - offset < data_len) {
        return false;
    }

    if (data != nullptr && data_len > 0) {
        memcpy(output + offset, data, data_len);
    }
    offset += data_len;

    *out_len = offset;
    return true;
}

bool BPv7Codec::decodeCBORBytes(const uint8_t* input, size_t input_len, uint8_t** data, size_t* data_len, size_t* consumed) {
    if (input == nullptr || input_len == 0) {
        return false;
    }

    uint8_t majorType = (input[0] >> 5) & 0x07;
    uint8_t addInfo = input[0] & 0x1F;

    // Must be byte string (major type 2)
    if (majorType != 2) {
        return false;
    }

    size_t length = 0;
    size_t offset = 1;

    if (addInfo < 24) {
        // Length 0-23 encoded in additional info
        length = addInfo;
    } else if (addInfo == 24) {
        // 1-byte length
        if (input_len < 2) return false;
        length = input[1];
        offset = 2;
    } else if (addInfo == 25) {
        // 2-byte length
        if (input_len < 3) return false;
        length = ((uint16_t)input[1] << 8) | input[2];
        offset = 3;
    } else if (addInfo == 26) {
        // 4-byte length
        if (input_len < 5) return false;
        length = ((uint32_t)input[1] << 24) | ((uint32_t)input[2] << 16) |
                 ((uint32_t)input[3] << 8) | input[4];
        offset = 5;
    } else {
        return false;
    }

    // Check if we have enough data
    if (input_len < offset + length) {
        return false;
    }

    // Return pointer to data (no copy - caller must copy if needed)
    *data = (uint8_t*)(input + offset);
    *data_len = length;
    *consumed = offset + length;

    return true;
}

bool BPv7Codec::encodeCBORArrayHeader(size_t array_size, uint8_t* output, size_t max_len, size_t* out_len) {
    if (output == nullptr || max_len == 0) {
        return false;
    }

    // Encode array header
    if (array_size < 24) {
        if (max_len < 1) return false;
        output[0] = CBOR_ARRAY | (uint8_t)array_size;
        *out_len = 1;
    } else if (array_size <= 0xFF) {
        if (max_len < 2) return false;
        output[0] = CBOR_ARRAY | 24;
        output[1] = (uint8_t)array_size;
        *out_len = 2;
    } else {
        // For embedded systems, limit array size
        return false;
    }

    return true;
}

uint16_t BPv7Codec::calculateCRC16(const uint8_t* data, size_t len) {
    // CRC16-X.25 (also known as CRC16-IBM)
    // Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
    // Initial value: 0xFFFF
    // Final XOR: 0xFFFF
    // Used by ION and TinyDTN reference implementation

    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408;  // 0x8408 is reversed polynomial
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFF;  // Final XOR
}

// ===== DEBUG FUNCTIONS =====

void BPv7Codec::debugPrintRawCBOR(const uint8_t* data, size_t len, const char* label) {
    Serial.printf("=== %s Data (%d bytes) ===\n", label, len);

    // Print hex dump in rows of 16 bytes
    for (size_t i = 0; i < len; i += 16) {
        Serial.printf("%04X: ", i);

        // Print hex values
        for (size_t j = 0; j < 16; j++) {
            if (i + j < len) {
                Serial.printf("%02X ", data[i + j]);
            } else {
                Serial.print("   ");
            }
        }

        Serial.print(" | ");

        // Print ASCII representation
        for (size_t j = 0; j < 16 && i + j < len; j++) {
            char c = data[i + j];
            if (c >= 32 && c <= 126) {
                Serial.print(c);
            } else {
                Serial.print('.');
            }
        }

        Serial.println();
    }

    // Print CBOR structure analysis
    Serial.println("CBOR Structure:");
    if (len > 0) {
        uint8_t major = (data[0] >> 5) & 0x07;
        uint8_t info = data[0] & 0x1F;

        Serial.printf("  First byte: 0x%02X\n", data[0]);
        Serial.printf("  Major type: %d (", major);
        switch (major) {
            case 0: Serial.print("unsigned int"); break;
            case 1: Serial.print("negative int"); break;
            case 2: Serial.print("byte string"); break;
            case 3: Serial.print("text string"); break;
            case 4: Serial.print("array"); break;
            case 5: Serial.print("map"); break;
            case 6: Serial.print("tag"); break;
            case 7: Serial.print("float/special"); break;
        }
        Serial.printf(")\n  Additional info: %d\n", info);
    }
    Serial.println("===");
}

void BPv7Codec::debugPrintBundle(const Bundle& bundle) {
    // Compact one-line bundle summary
    Serial.printf("Bundle: ipn:%lu.%lu->ipn:%lu.%lu seq=%lu time=%llu life=%lums payload=%dB\n",
                  bundle.primary.source.node_number,
                  bundle.primary.source.service_number,
                  bundle.primary.destination.node_number,
                  bundle.primary.destination.service_number,
                  bundle.primary.sequence_number,
                  bundle.primary.creation_timestamp,
                  bundle.primary.lifetime_ms,
                  bundle.payload.data_length);
}
