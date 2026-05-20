#ifndef BPV7_CODEC_H
#define BPV7_CODEC_H

#include <Arduino.h>
#include "Bundle.h"

/**
 * BPv7Codec - CBOR encoder/decoder for BPv7 bundles (RFC 8949)
 *
 * Handles serialization and deserialization of BPv7 bundles to/from
 * CBOR wire format as specified in RFC 9171.
 *
 * NOTE: This is a simplified implementation for embedded systems.
 * Full CBOR encoding will be implemented in Phase 2.
 */
class BPv7Codec {
public:
    /**
     * Encode a bundle to CBOR format
     *
     * @param bundle Bundle to encode
     * @param output Output buffer (must be pre-allocated)
     * @param max_len Maximum output buffer size
     * @param out_len Actual encoded length (output parameter)
     * @return true on success, false on error
     */
    static bool encode(const Bundle& bundle, uint8_t* output, size_t max_len, size_t* out_len);

    /**
     * Decode a CBOR-encoded bundle
     *
     * @param input Input CBOR data
     * @param input_len Length of input data
     * @param bundle Decoded bundle (output parameter)
     * @return true on success, false on error
     */
    static bool decode(const uint8_t* input, size_t input_len, Bundle& bundle);

    /**
     * Estimate encoded size of bundle
     *
     * @param bundle Bundle to estimate
     * @return Estimated size in bytes (approximate)
     */
    static size_t estimateSize(const Bundle& bundle);

    /**
     * Debug: Print raw CBOR data in hex format
     *
     * @param data Raw CBOR data
     * @param len Length of data
     * @param label Optional label for output
     */
    static void debugPrintRawCBOR(const uint8_t* data, size_t len, const char* label = "CBOR");

    /**
     * Debug: Print decoded bundle information
     *
     * @param bundle Bundle to print
     */
    static void debugPrintBundle(const Bundle& bundle);

private:
    /**
     * Encode PrimaryBlock to CBOR
     */
    static bool encodePrimaryBlock(const PrimaryBlock& primary, uint8_t* output, size_t max_len, size_t* out_len);

    /**
     * Encode PayloadBlock to CBOR
     */
    static bool encodePayloadBlock(const PayloadBlock& payload, uint8_t* output, size_t max_len, size_t* out_len);

    /**
     * Decode PrimaryBlock from CBOR
     */
    static bool decodePrimaryBlock(const uint8_t* input, size_t input_len, PrimaryBlock& primary, size_t* consumed);

    /**
     * Decode PayloadBlock from CBOR
     */
    static bool decodePayloadBlock(const uint8_t* input, size_t input_len, PayloadBlock& payload, size_t* consumed);

    /**
     * Encode EndpointID (IPN scheme) to CBOR
     * IPN uses CBOR array: [node_number, service_number]
     */
    static bool encodeEndpointID(const EndpointID& eid, uint8_t* output, size_t max_len, size_t* out_len);

    /**
     * Decode EndpointID (IPN scheme) from CBOR
     */
    static bool decodeEndpointID(const uint8_t* input, size_t input_len, EndpointID& eid, size_t* consumed);

    /**
     * Helper: Encode CBOR unsigned integer
     */
    static bool encodeCBORUint(uint64_t value, uint8_t* output, size_t max_len, size_t* out_len);

    /**
     * Helper: Decode CBOR unsigned integer
     */
    static bool decodeCBORUint(const uint8_t* input, size_t input_len, uint64_t* value, size_t* consumed);

    /**
     * Helper: Encode CBOR byte string
     */
    static bool encodeCBORBytes(const uint8_t* data, size_t data_len, uint8_t* output, size_t max_len, size_t* out_len);

    /**
     * Helper: Decode CBOR byte string
     */
    static bool decodeCBORBytes(const uint8_t* input, size_t input_len, uint8_t** data, size_t* data_len, size_t* consumed);

    /**
     * Helper: Encode CBOR array header
     */
    static bool encodeCBORArrayHeader(size_t array_size, uint8_t* output, size_t max_len, size_t* out_len);

    /**
     * Helper: Calculate CRC16 X.25 (used by ION)
     */
    static uint16_t calculateCRC16(const uint8_t* data, size_t len);
};

#endif // BPV7_CODEC_H
