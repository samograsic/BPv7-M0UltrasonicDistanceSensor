#include "LoRaCL.h"
#include <config.h>
#include <cpu_utils.h>  // For getTrueRandom()

LoRaCL::LoRaCL(int cs_pin, int dio0_pin, int rst_pin,
               float freq_mhz, float bw_khz,
               uint8_t sf, uint8_t cr, int8_t tx_power, uint8_t sync_word, uint8_t preamble_len)
    : radio()
    , cs_pin(cs_pin)
    , dio0_pin(dio0_pin)
    , rst_pin(rst_pin)
    , freq_mhz(freq_mhz)
    , bw_khz(bw_khz)
    , sf(sf)
    , cr(cr)
    , tx_power(tx_power)
    , sync_word(sync_word)
    , preamble_len(preamble_len)
    , initialized(false)
    , tx_record_count(0)
    , tx_record_index(0)
    , neighbor_count(0) {
    // Initialize TX records
    memset(tx_records, 0, sizeof(tx_records));
    memset(neighbors, 0, sizeof(neighbors));
}

bool LoRaCL::initialize() {
    Serial.printf("LoRa: Init %.2f MHz, SF%d, BW%.1f kHz, CR4/%d, %d dBm\n",
        freq_mhz, sf, bw_khz, cr, tx_power);

    // Initialize RadioLib driver with preamble length
    int state = radio.init(cs_pin, dio0_pin, rst_pin, -1,  // DIO1 not used
                          freq_mhz, bw_khz, sf, cr, tx_power, sync_word, preamble_len);

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa: ON");
        initialized = true;

        // Start listening for incoming bundles
        radio.startReceive();

        return true;
    } else {
        Serial.printf("LoRa: Init FAILED (code: %d)\n", state);
        initialized = false;
        return false;
    }
}

bool LoRaCL::transmit(const uint8_t* cbor_data, size_t cbor_len) {
    if (!initialized) {
        Serial.println("LoRa: TX failed - not initialized");
        return false;
    }

    if (cbor_len > 255) {
        Serial.printf("LoRa: TX failed - too large (%d > 255 bytes)\n", cbor_len);
        return false;
    }

    // Work with a mutable local copy so we can inject the extension block
    uint8_t tx_buf[255];
    size_t tx_len = cbor_len;
    memcpy(tx_buf, cbor_data, cbor_len);

    // Decode bundle for logging (before injection so decode sees clean bundle)
    Bundle bundle;
    bool decoded = BPv7Codec::decode(tx_buf, tx_len, bundle);

    // Inject LoRa extension block carrying our node EID and current TX duty cycle.
    // No-op if the extended bundle would exceed 255 bytes.
    EndpointID own_eid(DTN_NODE_NUMBER, 0);
    bool ext_added = addLoRaExtensionBlock(tx_buf, &tx_len, sizeof(tx_buf), own_eid);
    if (!ext_added) {
        Serial.printf("LoRa EXT: skipped (bundle %d bytes, would exceed 255 with ext block)\n", cbor_len);
    }

    // Record TX start time
    uint32_t tx_start = millis();

    // Transmit with collision avoidance if enabled
    int state;
#if LORA_COLLISION_AVOIDANCE_ENABLED
    state = transmitWithCollisionAvoidance(tx_buf, tx_len);
#else
    state = radio.transmit(tx_buf, tx_len);
#endif

    // Calculate TX duration
    uint32_t tx_duration = millis() - tx_start;

    if (state == RADIOLIB_ERR_NONE) {
        recordTxEvent(tx_duration);
        float dc = getTxDutyCycle();
        if (decoded) {
            Serial.printf(">> %lu.%lu -> %lu.%lu [%dB] (%lums) dc:%.1f%%\n",
                bundle.primary.source.node_number,
                bundle.primary.source.service_number,
                bundle.primary.destination.node_number,
                bundle.primary.destination.service_number,
                (int)tx_len, tx_duration, dc);
        } else {
            Serial.printf(">> [%dB] (%lums) dc:%.1f%%\n", (int)tx_len, tx_duration, dc);
        }
        radio.startReceive();
        return true;
    } else {
        Serial.printf(">> TX FAILED (code: %d)\n", state);
        radio.startReceive();
        return false;
    }
}

// ===== COLLISION AVOIDANCE (LISTEN BEFORE TALK / LBT) =====

bool LoRaCL::isChannelBusy() {
    return radio.getCurrentRSSI() > LORA_RSSI_THRESHOLD;
}

int LoRaCL::transmitWithCollisionAvoidance(const uint8_t* data, size_t len) {
    // Initial random backoff to spread out simultaneous transmitters
    uint32_t random_value = getTrueRandom();
    uint32_t backoff_ms = LORA_INITIAL_BACKOFF_MIN_MS +
                         (random_value % (LORA_INITIAL_BACKOFF_MAX_MS - LORA_INITIAL_BACKOFF_MIN_MS + 1));
    delay(backoff_ms);

    uint8_t retry_count = 0;
    uint32_t current_backoff_max = LORA_INITIAL_BACKOFF_MAX_MS;

    while (retry_count <= LORA_MAX_RETRIES) {
        int rssi = radio.getCurrentRSSI();
        if (rssi > LORA_RSSI_THRESHOLD) {
            retry_count++;
            if (retry_count > LORA_MAX_RETRIES) {
                Serial.printf("LBT: give up RSSI=%d (%d retries)\n", rssi, LORA_MAX_RETRIES);
                return -1;
            }
            current_backoff_max *= LORA_BACKOFF_MULTIPLIER;
            random_value = getTrueRandom();
            backoff_ms = LORA_INITIAL_BACKOFF_MIN_MS +
                        (random_value % (current_backoff_max - LORA_INITIAL_BACKOFF_MIN_MS + 1));
            Serial.printf("LBT: busy RSSI=%d (retry %d/%d)\n", rssi, retry_count, LORA_MAX_RETRIES);
            delay(backoff_ms);
            continue;
        }
        return radio.transmit(data, len);
    }
    return -1;
}

// ===== RECEPTION =====

bool LoRaCL::available() {
    if (!initialized) {
        return false;
    }

    return radio.available();
}

bool LoRaCL::receive(uint8_t* cbor_buffer, size_t max_len, size_t* out_len) {
    if (!initialized) {
        *out_len = 0;
        return false;
    }

    if (!radio.available()) {
        *out_len = 0;
        return false;
    }

    // Receive pure CBOR bundle data (no wrapper expected)
    size_t received_len = 0;
    int state = radio.receive(cbor_buffer, max_len, &received_len, 100);  // 100ms timeout

    if (state == RADIOLIB_ERR_NONE && received_len > 0) {
        int16_t rssi = radio.getRSSI();
        float snr = radio.getSNR();

        // Attempt to strip the LoRa extension block. The block is OPTIONAL —
        // bundles from nodes that do not add it are passed through unchanged.
        // If present: block is removed in-place and received_len is reduced.
        // If absent:  buffer and received_len are untouched; processing continues normally.
        uint32_t ext_node = 0;
        float ext_dc = 0.0f;
        bool had_ext = processLoRaExtensionBlock(cbor_buffer, &received_len, &ext_node, &ext_dc);

        *out_len = received_len;  // Caller always gets the final (possibly stripped) length

        // Decode clean bundle for logging
        Bundle bundle;
        bool decoded = BPv7Codec::decode(cbor_buffer, received_len, bundle);

        if (decoded) {
            bool is_new = false;
            if (bundle.primary.source.node_number != 0) {
                is_new = updateNeighbor(bundle.primary.source.node_number, rssi, snr,
                                        had_ext ? ext_dc : 0.0f);
            }
            // TXIPN: is who transmitted over LoRa (from extension block — may be a relay,
            // not the original bundle source). Shown first so the LoRa hop is immediately clear.
            if (had_ext) {
                Serial.printf("<< TXIPN:%lu %lu.%lu -> %lu.%lu [%dB] RSSI:%d SNR:%.1f dc=%u%%%s\n",
                    ext_node,
                    bundle.primary.source.node_number,
                    bundle.primary.source.service_number,
                    bundle.primary.destination.node_number,
                    bundle.primary.destination.service_number,
                    (int)received_len, rssi, snr,
                    (unsigned)ext_dc,
                    is_new ? " [new]" : "");
            } else {
                Serial.printf("<< %lu.%lu -> %lu.%lu [%dB] RSSI:%d SNR:%.1f\n",
                    bundle.primary.source.node_number,
                    bundle.primary.source.service_number,
                    bundle.primary.destination.node_number,
                    bundle.primary.destination.service_number,
                    (int)received_len, rssi, snr);
            }
        } else {
            if (had_ext) {
                Serial.printf("<< TXIPN:%lu [%dB] RSSI:%d SNR:%.1f dc=%u%% (decode failed)\n",
                    ext_node, (int)received_len, rssi, snr, (unsigned)ext_dc);
            } else {
                Serial.printf("<< [%dB] RSSI:%d SNR:%.1f (decode failed)\n",
                    (int)received_len, rssi, snr);
            }
        }

        // Restart listening
        radio.startReceive();

        return true;
    }

    *out_len = 0;

    // Restart listening even on failure
    if (state != RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.printf("<< RX FAILED (code: %d)\n", state);
    }
    radio.startReceive();

    return false;
}

int16_t LoRaCL::getLastRSSI() const {
    return radio.getRSSI();
}

float LoRaCL::getLastSNR() const {
    return radio.getSNR();
}

// ===== TX DUTY CYCLE TRACKING =====

void LoRaCL::recordTxEvent(uint32_t duration_ms) {
    // Clean old records first
    cleanOldTxRecords();

    // Add new record to circular buffer
    tx_records[tx_record_index].start_time_ms = millis();
    tx_records[tx_record_index].duration_ms = duration_ms;

    // Update index (circular buffer)
    tx_record_index = (tx_record_index + 1) % MAX_TX_RECORDS;

    // Update count (max at MAX_TX_RECORDS)
    if (tx_record_count < MAX_TX_RECORDS) {
        tx_record_count++;
    }

}

void LoRaCL::cleanOldTxRecords() {
    uint32_t current_time = millis();
    uint32_t one_hour_ago = current_time - (60UL * 60UL * 1000UL);  // 1 hour in milliseconds

    // Handle millis() rollover (after ~49 days)
    bool rollover = (current_time < (60UL * 60UL * 1000UL));

    uint8_t removed = 0;
    for (uint8_t i = 0; i < tx_record_count; i++) {
        // Check if record is older than 1 hour
        bool is_old = false;
        if (rollover) {
            // During rollover, keep records with high timestamps (before rollover) and low timestamps (after rollover)
            is_old = (tx_records[i].start_time_ms > one_hour_ago) && (tx_records[i].start_time_ms > current_time);
        } else {
            // Normal case
            is_old = (tx_records[i].start_time_ms < one_hour_ago);
        }

        if (is_old) {
            // Remove record by shifting array
            for (uint8_t j = i; j < tx_record_count - 1; j++) {
                tx_records[j] = tx_records[j + 1];
            }
            tx_record_count--;
            removed++;
            i--;  // Check this index again after shift
        }
    }

}

float LoRaCL::getTxDutyCycle() const {
    if (tx_record_count == 0) {
        return 0.0f;
    }

    // Calculate total TX time in last hour
    uint32_t total_tx_time = 0;
    for (uint8_t i = 0; i < tx_record_count; i++) {
        total_tx_time += tx_records[i].duration_ms;
    }

    // Calculate duty cycle percentage
    // 1 hour = 3,600,000 milliseconds
    float duty_cycle = (total_tx_time / 3600000.0f) * 100.0f;

    return duty_cycle;
}

uint32_t LoRaCL::getTxTimeLastHour() const {
    uint32_t total_tx_time = 0;
    for (uint8_t i = 0; i < tx_record_count; i++) {
        total_tx_time += tx_records[i].duration_ms;
    }
    return total_tx_time;
}

void LoRaCL::sleep() {
    radio.sleep();
}

void LoRaCL::idle() {
    radio.idle();
}

void LoRaCL::startReceive() {
    radio.startReceive();
}

// ===== BROADCAST BUNDLE HANDLING (CLA-LAYER ONLY) =====

bool LoRaCL::transmitBroadcastBeacon(const EndpointID& source_eid, uint64_t rtc_timestamp) {
    if (!initialized) {
        Serial.println("LoRa: Beacon TX failed - not initialized");
        return false;
    }

    // Create broadcast bundle (destination ipn:0.0)
    Bundle beacon;
    beacon.primary.version = 7;
    beacon.primary.bundle_proc_flags = 0;
    beacon.primary.crc_type = 1;  // CRC16 (required by ION)
    beacon.primary.destination = EndpointID(0, 0);  // Broadcast address
    beacon.primary.source = source_eid;
    beacon.primary.report_to = source_eid;
    beacon.primary.creation_timestamp = rtc_timestamp;
    beacon.primary.sequence_number = 0;
    beacon.primary.lifetime_ms = 3600000;  // 1 hour

    // Beacon payload: CBOR-encoded data
    // Format: [source_eid_node, source_eid_service, rtc_timestamp]
    // PLACEHOLDER: In future, add neighbor list with signal levels
    uint8_t payload[64];
    size_t payload_len = 0;

    // Simple CBOR array encoding: [node_number, service_number, timestamp]
    payload[payload_len++] = 0x83;  // CBOR array of 3 elements

    // Encode node number (uint32)
    payload[payload_len++] = 0x1A;  // Major type 0 (unsigned int), additional info 26 (4 bytes)
    payload[payload_len++] = (source_eid.node_number >> 24) & 0xFF;
    payload[payload_len++] = (source_eid.node_number >> 16) & 0xFF;
    payload[payload_len++] = (source_eid.node_number >> 8) & 0xFF;
    payload[payload_len++] = source_eid.node_number & 0xFF;

    // Encode service number (uint32)
    payload[payload_len++] = 0x1A;
    payload[payload_len++] = (source_eid.service_number >> 24) & 0xFF;
    payload[payload_len++] = (source_eid.service_number >> 16) & 0xFF;
    payload[payload_len++] = (source_eid.service_number >> 8) & 0xFF;
    payload[payload_len++] = source_eid.service_number & 0xFF;

    // Encode timestamp (uint64)
    payload[payload_len++] = 0x1B;
    payload[payload_len++] = (rtc_timestamp >> 56) & 0xFF;
    payload[payload_len++] = (rtc_timestamp >> 48) & 0xFF;
    payload[payload_len++] = (rtc_timestamp >> 40) & 0xFF;
    payload[payload_len++] = (rtc_timestamp >> 32) & 0xFF;
    payload[payload_len++] = (rtc_timestamp >> 24) & 0xFF;
    payload[payload_len++] = (rtc_timestamp >> 16) & 0xFF;
    payload[payload_len++] = (rtc_timestamp >> 8) & 0xFF;
    payload[payload_len++] = rtc_timestamp & 0xFF;

    // Set payload using PayloadBlock's setData method
    beacon.payload.block_type = 1;  // Payload block
    beacon.payload.block_number = 1;
    beacon.payload.block_proc_flags = 0;
    beacon.payload.crc_type = 0;
    beacon.payload.setData(payload, payload_len);

    // Encode bundle to CBOR
    uint8_t cbor_buffer[255];
    size_t cbor_len = 0;
    if (!BPv7Codec::encode(beacon, cbor_buffer, sizeof(cbor_buffer), &cbor_len)) {
        Serial.println("LoRa: Beacon encode failed");
        return false;
    }

    // Record TX start time
    uint32_t tx_start = millis();

    // Transmit with collision avoidance if enabled
    int state;
#if LORA_COLLISION_AVOIDANCE_ENABLED
    state = transmitWithCollisionAvoidance(cbor_buffer, cbor_len);
#else
    state = radio.transmit(cbor_buffer, cbor_len);
#endif

    // Calculate TX duration
    uint32_t tx_duration = millis() - tx_start;

    if (state == RADIOLIB_ERR_NONE) {
        recordTxEvent(tx_duration);
        float dc = getTxDutyCycle();
        Serial.printf(">> BEACON %lu.%lu [%dB] (%lums) dc:%.1f%%\n",
            source_eid.node_number, source_eid.service_number,
            (int)cbor_len, tx_duration, dc);
        radio.startReceive();
        return true;
    } else {
        Serial.printf(">> BEACON TX FAILED (code: %d)\n", state);
        radio.startReceive();
        return false;
    }
}

bool LoRaCL::processBroadcastBundle(const uint8_t* cbor_data, size_t cbor_len, int16_t rssi, float snr) {
    // Decode bundle to check if it's a broadcast
    Bundle bundle;
    if (!BPv7Codec::decode(cbor_data, cbor_len, bundle)) {
        // Not a valid bundle, ignore
        return false;
    }

    // Check if destination is broadcast address (ipn:0.0)
    if (bundle.primary.destination.node_number != 0 || bundle.primary.destination.service_number != 0) {
        // Not a broadcast bundle
        return false;
    }

    // Clean output: << BEACON src [payload_len bytes] RSSI: -XX dBm
    Serial.printf("<< BEACON %lu.%lu [%d bytes] RSSI: %d dBm, SNR: %.1f dB\n",
        bundle.primary.source.node_number,
        bundle.primary.source.service_number,
        bundle.payload.data_length,
        rssi, snr);

    // Track beacon sender as neighbor
    if (bundle.primary.source.node_number != 0) {
        updateNeighbor(bundle.primary.source.node_number, rssi, snr);
    }

    // Broadcast bundle processed - don't pass to Bundle Manager
    return true;
}

// ===== NEIGHBOR TRACKING =====

void LoRaCL::cleanExpiredNeighbors() {
    uint32_t now = millis();
    const uint32_t EXPIRY_MS = 24UL * 3600UL * 1000UL;  // 24 hours

    for (uint8_t i = 0; i < neighbor_count; ) {
        if ((now - neighbors[i].last_seen_ms) > EXPIRY_MS) {
            // Remove by swapping with last entry
            neighbors[i] = neighbors[neighbor_count - 1];
            neighbor_count--;
        } else {
            i++;
        }
    }
}

bool LoRaCL::updateNeighbor(uint32_t node_number, int16_t rssi, float snr, float duty_cycle) {
    cleanExpiredNeighbors();

    for (uint8_t i = 0; i < neighbor_count; i++) {
        if (neighbors[i].node_number == node_number) {
            neighbors[i].last_seen_ms = millis();
            neighbors[i].rssi = rssi;
            neighbors[i].snr = snr;
            if (duty_cycle > 0.0f) {
                neighbors[i].tx_duty_cycle = duty_cycle;
            }
            return false;  // existing neighbor
        }
    }

    if (neighbor_count < MAX_LORA_NEIGHBORS) {
        neighbors[neighbor_count] = { node_number, millis(), rssi, snr, duty_cycle };
        neighbor_count++;
    } else {
        uint8_t oldest_idx = 0;
        for (uint8_t i = 1; i < neighbor_count; i++) {
            if (neighbors[i].last_seen_ms < neighbors[oldest_idx].last_seen_ms) {
                oldest_idx = i;
            }
        }
        neighbors[oldest_idx] = { node_number, millis(), rssi, snr, duty_cycle };
    }
    return true;  // new neighbor
}

// ===== LORA EXTENSION BLOCK HANDLING =====
//
// Block type: 192 (0xC0) — private/experimental range, RFC 9171 Section 9.1
// Block processing flags: 0x10 — discard block if unknown, keep bundle
// CRC: none (LoRa PHY layer provides its own CRC)
//
// Wire layout — variable size (11-16 bytes depending on node number and duty cycle):
//
//   Bytes  Value           Description
//   1      0x85            CBOR array(5) — canonical block
//   2      0x18 0xC0       block_type = 192
//   1      0x02            block_number = 2
//   1      0x10            block_proc_flags (discard if unknown)
//   1      0x00            crc_type = none
//   1-2    0x4N / 0x58 NN  CBOR bstr(content_len)
//   ----   [CBOR content]:
//   1      0x83            CBOR array(3)
//   1-5    <uint>          node EID (CBOR variable-length uint — 1 byte for node<24, 5 for 32-bit)
//   1      0x00            fragment flag (0=none, 1-254=seg nr, 255=last)
//   1-2    <uint>          TX duty cycle % (0-100, 1-byte for <24%, 2-byte for 24-100%)
//
// Typical sizes:
//   Node < 24,  dc < 24%  →  6+1+4 = 11 bytes
//   Node 32-bit, dc < 24%  →  6+1+8 = 15 bytes   (this network: ~15 bytes)
//   Node 32-bit, dc 24-100% → 6+1+9 = 16 bytes

// Minimal CBOR uint helpers (file-local, no external dependency)
static size_t lora_cbor_put_uint(uint8_t* buf, uint32_t v) {
    if (v <= 23)       { buf[0] = (uint8_t)v; return 1; }
    if (v <= 0xFF)     { buf[0] = 0x18; buf[1] = (uint8_t)v; return 2; }
    if (v <= 0xFFFF)   { buf[0] = 0x19; buf[1] = v>>8; buf[2] = v&0xFF; return 3; }
    buf[0]=0x1A; buf[1]=v>>24; buf[2]=v>>16; buf[3]=v>>8; buf[4]=v&0xFF; return 5;
}

static size_t lora_cbor_get_uint(const uint8_t* buf, size_t avail, uint32_t* v) {
    if (avail < 1) return 0;
    uint8_t info = buf[0] & 0x1F;
    if ((buf[0] >> 5) != 0) return 0;  // Not a CBOR uint
    if (info <= 23) { *v = info; return 1; }
    if (info == 24 && avail >= 2) { *v = buf[1]; return 2; }
    if (info == 25 && avail >= 3) { *v = ((uint32_t)buf[1]<<8)|buf[2]; return 3; }
    if (info == 26 && avail >= 5) {
        *v = ((uint32_t)buf[1]<<24)|((uint32_t)buf[2]<<16)|((uint32_t)buf[3]<<8)|buf[4];
        return 5;
    }
    return 0;
}

bool LoRaCL::addLoRaExtensionBlock(uint8_t* cbor_data, size_t* cbor_len, size_t max_len,
                                   const EndpointID& transmitter_eid) {
    // Bundle must end with 0xFF (CBOR indefinite-array break)
    if (*cbor_len < 2 || cbor_data[*cbor_len - 1] != 0xFF) {
        return false;
    }

    // Build CBOR content into a temp buffer
    uint8_t content[12];
    size_t  clen = 0;

    content[clen++] = 0x83;  // CBOR array(3)
    clen += lora_cbor_put_uint(content + clen, transmitter_eid.node_number);
    content[clen++] = 0x00;  // fragment flag = 0 (not fragmented)
    float dc_f = getTxDutyCycle();
    uint8_t dc = (dc_f >= 100.0f) ? 100u : (uint8_t)(dc_f + 0.5f);
    clen += lora_cbor_put_uint(content + clen, dc);

    // Build the full canonical block: 6-byte CBOR wrapper + bstr header + content
    uint8_t ext[20];
    size_t  elen = 0;

    ext[elen++] = 0x85;   // CBOR array(5)
    ext[elen++] = 0x18;   // uint 1-byte follows
    ext[elen++] = 0xC0;   // 192 = LORA_EXT_BLOCK_TYPE
    ext[elen++] = 0x02;   // block_number = 2
    ext[elen++] = 0x10;   // block_proc_flags = 0x10 (discard if unknown)
    ext[elen++] = 0x00;   // crc_type = none

    // CBOR bstr header (clen will always be < 24 for our content)
    ext[elen++] = 0x40 | (uint8_t)clen;

    memcpy(ext + elen, content, clen);
    elen += clen;

    // Check headroom and insert before the final 0xFF
    if (*cbor_len + elen > max_len) {
        return false;
    }

    cbor_data[*cbor_len - 1 + elen] = 0xFF;
    memcpy(cbor_data + *cbor_len - 1, ext, elen);
    *cbor_len += elen;
    return true;
}

bool LoRaCL::processLoRaExtensionBlock(uint8_t* cbor_data, size_t* cbor_len,
                                        uint32_t* out_node, float* out_duty_cycle) {
    *out_node       = 0;
    *out_duty_cycle = 0.0f;

    // Minimum possible block: 6-byte wrapper + 1-byte bstr header + 4-byte content = 11
    if (*cbor_len < 13) {
        return false;
    }

    // Scan for extension block signature: array(5), uint=192
    // Pattern: 0x85 0x18 0xC0 — start at offset 1 (skip 0x9F indefinite-array open)
    for (size_t i = 1; i + 11 <= *cbor_len; i++) {
        if (cbor_data[i]   != 0x85) continue;
        if (cbor_data[i+1] != 0x18) continue;
        if (cbor_data[i+2] != 0xC0) continue;
        // i+3=block_number, i+4=flags, i+5=crc_type (we don't validate these)

        // Parse bstr header at i+6
        uint8_t bh = cbor_data[i+6];
        if ((bh >> 5) != 2) continue;        // Major type 2 = byte string
        size_t content_len, bstr_hdr_len;
        if ((bh & 0x1F) <= 23) {
            content_len  = bh & 0x1F;
            bstr_hdr_len = 1;
        } else {
            continue;  // Unexpectedly long content — not our block
        }

        size_t block_len     = 6 + bstr_hdr_len + content_len;
        size_t content_start = i + 6 + bstr_hdr_len;

        if (i + block_len > *cbor_len) continue;

        // Parse CBOR content: array(3), node_uint, frag_flag, dc_uint
        const uint8_t* c   = cbor_data + content_start;
        size_t         rem = content_len;

        if (rem < 1 || c[0] != 0x83) continue;  // Must be array(3)
        c++; rem--;

        uint32_t node_val = 0;
        size_t   rd = lora_cbor_get_uint(c, rem, &node_val);
        if (rd == 0) continue;
        c += rd; rem -= rd;

        if (rem < 1) continue;  // fragment flag
        c++; rem--;             // skip fragment flag (reserved)

        uint32_t dc_val = 0;
        rd = lora_cbor_get_uint(c, rem, &dc_val);
        if (rd == 0) continue;

        *out_node       = node_val;
        *out_duty_cycle = (float)dc_val;

        // Remove block from buffer
        memmove(cbor_data + i, cbor_data + i + block_len, *cbor_len - i - block_len);
        *cbor_len -= block_len;
        return true;
    }

    return false;  // No extension block present — buffer and *cbor_len are unchanged
}
