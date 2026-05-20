#include "BundleManager.h"
#include "BundleService.h"

// DTN Epoch: 2000-01-01 00:00:00 UTC
// Unix Epoch: 1970-01-01 00:00:00 UTC
// Difference: 946684800 seconds
#define DTN_EPOCH_OFFSET 946684800000ULL  // milliseconds

//=============================================================================
// RamBundleStore Implementation
//=============================================================================

RamBundleStore::RamBundleStore() : bundle_count(0) {
}

bool RamBundleStore::store(const Bundle& bundle) {
    // Check for duplicates first
    if (exists(bundle)) {
        Serial.println("BundleStore: Duplicate bundle detected, not storing");
        return false;
    }

    // If store is full, implement FIFO - remove oldest bundle (index 0)
    if (bundle_count >= MAX_BUNDLE_STORE_SIZE) {
        Serial.printf("BundleStore: FULL (%d/%d) — FIFO removing id=%lu\n",
                     bundle_count, MAX_BUNDLE_STORE_SIZE, bundles[0].metadata.local_id);
        remove(0);  // Remove oldest (first) bundle
    }

    // Store bundle at the end
    bundles[bundle_count] = bundle;
    bundle_count++;

    Serial.printf("BundleStore: +id=%lu seq=%lu (%d/%d)\n",
                  bundle.metadata.local_id,
                  bundle.primary.sequence_number,
                  bundle_count,
                  MAX_BUNDLE_STORE_SIZE);

    return true;
}

bool RamBundleStore::get(size_t index, Bundle& bundle) const {
    if (index >= bundle_count) {
        return false;
    }

    bundle = bundles[index];
    return true;
}

bool RamBundleStore::remove(size_t index) {
    if (index >= bundle_count) {
        return false;
    }

    // Shift bundles down
    for (size_t i = index; i < bundle_count - 1; i++) {
        bundles[i] = bundles[i + 1];
    }

    bundle_count--;
    return true;
}

bool RamBundleStore::exists(const Bundle& bundle) const {
    for (size_t i = 0; i < bundle_count; i++) {
        if (bundles[i].primary.source == bundle.primary.source &&
            bundles[i].primary.creation_timestamp == bundle.primary.creation_timestamp &&
            bundles[i].primary.sequence_number == bundle.primary.sequence_number) {
            return true;
        }
    }
    return false;
}

size_t RamBundleStore::removeExpired(uint64_t current_time_ms) {
    size_t removed = 0;
    size_t i = 0;

    while (i < bundle_count) {
        if (bundles[i].isExpired(current_time_ms)) {
            Serial.printf("BundleStore: Removing expired bundle from ipn:%lu.%lu\n",
                         bundles[i].primary.source.node_number,
                         bundles[i].primary.source.service_number);
            remove(i);
            removed++;
            // Don't increment i, since we shifted array down
        } else {
            i++;
        }
    }

    if (removed > 0) {
        Serial.printf("BundleStore: Removed %d expired bundles\n", removed);
    }

    return removed;
}

//=============================================================================
// BundleManager Implementation
//=============================================================================

BundleManager::BundleManager()
    : store()
    , local_eid(0, 0)
    , sequence_counter(0)
    , time_source_callback(nullptr)
    , cla_count(0)
    , service_count(0)
    , tx_record_count(0) {
    memset(clas, 0, sizeof(clas));
    memset(services, 0, sizeof(services));
    memset(tx_records, 0, sizeof(tx_records));
}

bool BundleManager::initialize() {
    Serial.println("BundleManager: Initializing...");

    // Reset state
    sequence_counter = 0;
    cla_count = 0;
    service_count = 0;
    tx_record_count = 0;

    Serial.println("BundleManager: Initialized successfully");
    return true;
}

void BundleManager::setTimeSource(uint64_t (*callback)()) {
    time_source_callback = callback;
    Serial.println("BundleManager: Time source callback registered");
}

bool BundleManager::registerCLA(IConvergenceLayer* cla) {
    if (cla == nullptr) {
        Serial.println("BundleManager: ERROR - Cannot register null CLA");
        return false;
    }

    if (cla_count >= 4) {
        Serial.println("BundleManager: ERROR - Maximum CLAs registered");
        return false;
    }

    clas[cla_count] = cla;
    Serial.printf("BundleManager: Registered CLA '%s' (index %d)\n",
                  cla->getName(), cla_count);
    cla_count++;

    return true;
}

bool BundleManager::registerService(IBundleService* service) {
    if (service == nullptr) {
        Serial.println("BundleManager: ERROR - Cannot register null service");
        return false;
    }

    if (service_count >= MAX_SERVICES) {
        Serial.println("BundleManager: ERROR - Maximum services registered");
        return false;
    }

    services[service_count] = service;
    Serial.printf("BundleManager: Registered service '%s' on port %lu (index %d)\n",
                  service->getName(), service->getServiceNumber(), service_count);
    service_count++;

    return true;
}

void BundleManager::setLocalEID(uint32_t node_number, uint32_t service_number) {
    local_eid.node_number = node_number;
    local_eid.service_number = service_number;
    Serial.printf("BundleManager: Set local EID to ipn:%lu.%lu\n",
                  node_number, service_number);
}

uint64_t BundleManager::getCurrentDtnTime() const {
    // Use registered time source callback if available
    if (time_source_callback != nullptr) {
        return time_source_callback();
    }

    // Fallback to millis() if no time source registered (not recommended for production)
    Serial.println("WARNING: BundleManager using millis() as time source - bundles may expire immediately!");
    return millis();
}

uint64_t BundleManager::getNextSequenceNumber() {
    return sequence_counter++;
}

bool BundleManager::createBundle(const EndpointID& destination,
                                 const uint8_t* payload_data,
                                 size_t payload_len,
                                 uint32_t lifetime_ms) {
    // Create bundle
    Bundle bundle;

    // Set primary block
    bundle.primary.version = 7;
    bundle.primary.bundle_proc_flags = 0;
    bundle.primary.crc_type = 1;  // CRC16 (required by ION)
    bundle.primary.destination = destination;
    bundle.primary.source = local_eid;
    bundle.primary.report_to = local_eid;
    bundle.primary.creation_timestamp = getCurrentDtnTime();
    bundle.primary.sequence_number = getNextSequenceNumber();
    bundle.primary.lifetime_ms = lifetime_ms;
    bundle.primary.fragment_offset = 0;
    bundle.primary.total_data_length = 0;

    // Set payload
    bundle.payload.setData(payload_data, payload_len);

    // Set metadata
    bundle.metadata.local_id = bundle.primary.sequence_number;
    bundle.metadata.received_at_ms = getCurrentDtnTime();
    bundle.metadata.expires_at_ms = bundle.primary.creation_timestamp + lifetime_ms;
    bundle.metadata.hop_count = 0;
    bundle.metadata.max_hops = 32;
    bundle.metadata.locally_generated = true;
    bundle.metadata.forwarded = false;

    // Check size constraint
    size_t estimated_size = BPv7Codec::estimateSize(bundle);
    if (estimated_size > MAX_LORA_CBOR_SIZE) {
        Serial.printf("BundleManager: ERROR - Bundle too large (%d > %d bytes)\n",
                     estimated_size, MAX_LORA_CBOR_SIZE);
        return false;
    }

    // Before storing, check if FIFO will remove a bundle and clean its TX record
    if (store.count() >= MAX_BUNDLE_STORE_SIZE) {
        // Get the oldest bundle that will be removed
        Bundle oldest;
        if (store.get(0, oldest)) {
            uint32_t removed_local_id = oldest.metadata.local_id;
            // Remove its TX record proactively
            removeTxRecord(removed_local_id);
        }
    }

    // Store bundle (may trigger FIFO removal)
    return store.store(bundle);
}

bool BundleManager::createBundle(const EndpointID& destination,
                                 const uint8_t* payload_data,
                                 size_t payload_len,
                                 uint32_t lifetime_ms,
                                 uint32_t source_service_number) {
    // Create bundle
    Bundle bundle;

    // Set primary block
    bundle.primary.version = 7;
    bundle.primary.bundle_proc_flags = 0;
    bundle.primary.crc_type = 1;  // CRC16 (required by ION)
    bundle.primary.destination = destination;

    // Use custom source service number
    bundle.primary.source = local_eid;
    bundle.primary.source.service_number = source_service_number;

    bundle.primary.report_to = local_eid;
    bundle.primary.creation_timestamp = getCurrentDtnTime();
    bundle.primary.sequence_number = getNextSequenceNumber();
    bundle.primary.lifetime_ms = lifetime_ms;
    bundle.primary.fragment_offset = 0;
    bundle.primary.total_data_length = 0;

    // Set payload
    bundle.payload.setData(payload_data, payload_len);

    // Set metadata
    bundle.metadata.local_id = bundle.primary.sequence_number;
    bundle.metadata.received_at_ms = getCurrentDtnTime();
    bundle.metadata.expires_at_ms = bundle.primary.creation_timestamp + lifetime_ms;
    bundle.metadata.hop_count = 0;
    bundle.metadata.max_hops = 32;
    bundle.metadata.locally_generated = true;
    bundle.metadata.forwarded = false;

    // Check size constraint
    size_t estimated_size = BPv7Codec::estimateSize(bundle);
    if (estimated_size > MAX_LORA_CBOR_SIZE) {
        Serial.printf("BundleManager: ERROR - Bundle too large (%d > %d bytes)\n",
                     estimated_size, MAX_LORA_CBOR_SIZE);
        return false;
    }

    // Before storing, check if FIFO will remove a bundle and clean its TX record
    if (store.count() >= MAX_BUNDLE_STORE_SIZE) {
        // Get the oldest bundle that will be removed
        Bundle oldest;
        if (store.get(0, oldest)) {
            uint32_t removed_local_id = oldest.metadata.local_id;
            // Remove its TX record proactively
            removeTxRecord(removed_local_id);
        }
    }

    // Store bundle (may trigger FIFO removal)
    return store.store(bundle);
}

bool BundleManager::processReceivedBundle(const uint8_t* cbor_data,
                                         size_t cbor_len,
                                         int16_t rssi,
                                         float snr,
                                         const char* cla_name) {
    const char* via = cla_name ? cla_name : "?";

    // Decode CBOR
    Bundle bundle;
    if (!BPv7Codec::decode(cbor_data, cbor_len, bundle)) {
        Serial.printf("BM RX: %s [%dB] -> DECODE_FAIL\n", via, cbor_len);
        return false;
    }

    // Set metadata
    bundle.metadata.local_id = bundle.primary.sequence_number;
    bundle.metadata.received_at_ms = getCurrentDtnTime();
    bundle.metadata.expires_at_ms = bundle.primary.creation_timestamp + bundle.primary.lifetime_ms;
    bundle.metadata.locally_generated = false;
    bundle.metadata.forwarded = false;

    // Track which CLA received this bundle (for epidemic forwarding)
    if (cla_name != nullptr) {
        for (uint8_t i = 0; i < cla_count; i++) {
            if (strcmp(clas[i]->getName(), cla_name) == 0) {
                bundle.metadata.received_via_cla = i;
                break;
            }
        }
    } else {
        bundle.metadata.received_via_cla = 0xFF;
    }

    // Check if bundle destination node matches local node
    if (bundle.primary.destination.node_number == local_eid.node_number) {
        const char* svc = deliverToLocalService(bundle);
        Serial.printf("BM RX: %s ipn:%lu.%lu->%lu.%lu [%dB] -> %s\n",
            via,
            bundle.primary.source.node_number, bundle.primary.source.service_number,
            bundle.primary.destination.node_number, bundle.primary.destination.service_number,
            cbor_len, svc ? svc : "no-service");
        return true;
    }

    // Check expiration
    if (bundle.isExpired(getCurrentDtnTime())) {
        Serial.printf("BM RX: %s ipn:%lu.%lu->%lu.%lu [%dB] -> expired\n",
            via,
            bundle.primary.source.node_number, bundle.primary.source.service_number,
            bundle.primary.destination.node_number, bundle.primary.destination.service_number,
            cbor_len);
        return false;
    }

    // Store for forwarding
    if (store.count() >= MAX_BUNDLE_STORE_SIZE) {
        Bundle oldest;
        if (store.get(0, oldest)) {
            removeTxRecord(oldest.metadata.local_id);
        }
    }

    bool stored = store.store(bundle);
    Serial.printf("BM RX: %s ipn:%lu.%lu->%lu.%lu [%dB] -> %s\n",
        via,
        bundle.primary.source.node_number, bundle.primary.source.service_number,
        bundle.primary.destination.node_number, bundle.primary.destination.service_number,
        cbor_len, stored ? "store" : "store-FULL");
    return stored;
}

uint16_t BundleManager::forwardBundles() {
    if (cla_count == 0) {
        return 0;
    }

    uint16_t transmitted = 0;
    uint8_t cbor_buffer[MAX_LORA_CBOR_SIZE];

    Serial.printf("BundleManager: Forwarding bundles (store has %d bundles, %d CLAs)\n",
                  store.count(), cla_count);

    // Iterate through all stored bundles
    for (size_t bundle_idx = 0; bundle_idx < store.count(); bundle_idx++) {
        Bundle bundle;
        if (!store.get(bundle_idx, bundle)) {
            continue;
        }

        uint32_t bundle_local_id = bundle.metadata.local_id;

        Serial.printf("BundleManager: Processing bundle %lu (to ipn:%lu.%lu)\n",
                     bundle_local_id,
                     bundle.primary.destination.node_number,
                     bundle.primary.destination.service_number);

        // Try to send via each CLA
        for (uint8_t cla_idx = 0; cla_idx < cla_count; cla_idx++) {
            IConvergenceLayer* cla = clas[cla_idx];

            // Skip if already sent via this CLA
            if (isSentViaCLA(bundle_local_id, cla_idx)) {
                Serial.printf("  Already sent via %s, skipping\n", cla->getName());
                continue;
            }

            // Check MTU
            size_t estimated_size = BPv7Codec::estimateSize(bundle);
            if (estimated_size > cla->getMTU()) {
                Serial.printf("  Bundle too large for %s MTU (%d > %d), skipping\n",
                            cla->getName(), estimated_size, cla->getMTU());
                continue;
            }

            // Encode to CBOR
            size_t cbor_len = 0;
            if (!BPv7Codec::encode(bundle, cbor_buffer, sizeof(cbor_buffer), &cbor_len)) {
                Serial.printf("  Failed to encode bundle for %s\n", cla->getName());
                continue;
            }

            // Transmit
            Serial.printf("  Transmitting via %s (%d bytes CBOR)...\n",
                         cla->getName(), cbor_len);

            if (cla->transmit(cbor_buffer, cbor_len)) {
                Serial.printf("  SUCCESS: Transmitted via %s\n", cla->getName());
                markSentViaCLA(bundle_local_id, cla_idx);
                transmitted++;
            } else {
                Serial.printf("  FAILED: Transmission via %s failed\n", cla->getName());
            }
        }
    }

    Serial.printf("BundleManager: Forwarding complete - transmitted %d bundles\n", transmitted);
    return transmitted;
}

uint16_t BundleManager::forwardBundlesViaCLA(const char* cla_name) {
    if (cla_count == 0) {
        return 0;
    }

    // Find the specified CLA
    uint8_t target_cla_idx = 0xFF;
    for (uint8_t i = 0; i < cla_count; i++) {
        if (strcmp(clas[i]->getName(), cla_name) == 0) {
            target_cla_idx = i;
            break;
        }
    }

    if (target_cla_idx == 0xFF) {
        Serial.printf("BundleManager: CLA '%s' not found\n", cla_name);
        return 0;
    }

    IConvergenceLayer* cla = clas[target_cla_idx];
    uint16_t transmitted = 0;
    uint16_t skipped_already_sent = 0;
    uint16_t skipped_loop_prevention = 0;
    uint16_t skipped_mtu = 0;
    uint16_t skipped_encode_fail = 0;
    uint16_t total_bundles = store.count();
    uint8_t cbor_buffer[MAX_LORA_CBOR_SIZE];

    Serial.printf("BundleManager: Forwarding via %s (store has %d bundles)...\n", cla_name, total_bundles);

    // Iterate through all stored bundles
    for (size_t bundle_idx = 0; bundle_idx < total_bundles; bundle_idx++) {
        Bundle bundle;
        if (!store.get(bundle_idx, bundle)) {
            continue;
        }

        uint32_t bundle_local_id = bundle.metadata.local_id;

        // EPIDEMIC FORWARDING RULE: For LTE, skip bundles that were received via LTE (prevent loops)
        // For LoRa, forward all bundles (including those from LTE)
        if (strcmp(cla_name, "LTE") == 0 && bundle.metadata.received_via_cla == target_cla_idx) {
            Serial.printf("  Bundle %lu: SKIP (received via LTE, loop prevention)\n", bundle_local_id);
            skipped_loop_prevention++;
            continue;
        }

        // Skip if already sent via this CLA
        if (isSentViaCLA(bundle_local_id, target_cla_idx)) {
            Serial.printf("  Bundle %lu: SKIP (already sent via %s)\n", bundle_local_id, cla_name);
            skipped_already_sent++;
            continue;
        }

        // Check MTU
        size_t estimated_size = BPv7Codec::estimateSize(bundle);
        if (estimated_size > cla->getMTU()) {
            skipped_mtu++;
            continue;
        }

        // Encode to CBOR
        size_t cbor_len = 0;
        if (!BPv7Codec::encode(bundle, cbor_buffer, sizeof(cbor_buffer), &cbor_len)) {
            skipped_encode_fail++;
            continue;
        }

        // Transmit
        Serial.printf("  [%d/%d] TX bundle %lu to ipn:%lu.%lu via %s (%d bytes)...",
                     transmitted + 1, total_bundles - skipped_already_sent - skipped_loop_prevention - skipped_mtu - skipped_encode_fail,
                     bundle_local_id,
                     bundle.primary.destination.node_number,
                     bundle.primary.destination.service_number,
                     cla->getName(), cbor_len);

        if (cla->transmit(cbor_buffer, cbor_len)) {
            Serial.println(" OK");
            markSentViaCLA(bundle_local_id, target_cla_idx);
            transmitted++;

            // Wait 500ms between bundles to prevent concatenation at receiver
            delay(500);
        } else {
            Serial.println(" FAIL");
        }
    }

    // Summary output
    Serial.printf("BundleManager: %s forwarding complete: %d sent", cla_name, transmitted);
    if (skipped_already_sent > 0) Serial.printf(", %d already sent", skipped_already_sent);
    if (skipped_loop_prevention > 0) Serial.printf(", %d loop-prevented", skipped_loop_prevention);
    if (skipped_mtu > 0) Serial.printf(", %d MTU-exceed", skipped_mtu);
    if (skipped_encode_fail > 0) Serial.printf(", %d encode-fail", skipped_encode_fail);
    Serial.println();

    return transmitted;
}

uint16_t BundleManager::removeExpiredBundles() {
    uint64_t current_time = getCurrentDtnTime();

    // Before removing expired bundles, collect their local_ids to clean TX records
    uint32_t expired_local_ids[MAX_BUNDLE_STORE_SIZE];
    uint8_t expired_count = 0;

    for (size_t i = 0; i < store.count(); i++) {
        Bundle bundle;
        if (store.get(i, bundle) && bundle.isExpired(current_time)) {
            if (expired_count < MAX_BUNDLE_STORE_SIZE) {
                expired_local_ids[expired_count++] = bundle.metadata.local_id;
            }
        }
    }

    // Remove expired bundles from store
    size_t removed = store.removeExpired(current_time);

    // Clean up TX records for removed bundles
    for (uint8_t i = 0; i < expired_count; i++) {
        removeTxRecord(expired_local_ids[i]);
    }

    return removed;
}

uint16_t BundleManager::removeForwardedBundles() {
    uint16_t removed = 0;
    uint16_t checked = 0;

    Serial.printf("BundleManager: Checking %d bundles for removal...\n", store.count());

    // Iterate backwards to safely remove while iterating
    for (int i = store.count() - 1; i >= 0; i--) {
        Bundle bundle;
        if (!store.get(i, bundle)) {
            continue;
        }

        uint32_t bundle_local_id = bundle.metadata.local_id;
        BundleTransmitRecord* record = getTxRecord(bundle_local_id);
        checked++;

        // Skip if no TX record (bundle never sent)
        if (record == nullptr) {
            continue;
        }

        // Determine required CLAs for epidemic forwarding
        // Rule: LoRa forwards all bundles
        // Rule: LTE forwards all bundles EXCEPT those received via LTE
        bool lora_required = true;
        bool lte_required = (bundle.metadata.received_via_cla != 1); // CLA index 1 is LTE

        // Check if bundle has been sent via all required CLAs
        uint8_t lora_cla_idx = 0;  // LoRa is CLA index 0
        uint8_t lte_cla_idx = 1;   // LTE is CLA index 1

        bool lora_sent = (record->sent_via_cla_mask & (1 << lora_cla_idx)) != 0;
        bool lte_sent = (record->sent_via_cla_mask & (1 << lte_cla_idx)) != 0;

        // Bundle is fully forwarded if all required CLAs have sent it
        bool fully_forwarded = (!lora_required || lora_sent) && (!lte_required || lte_sent);

        Serial.printf("  Bundle %lu: rx_via=%d LoRa:%s%s LTE:%s%s -> %s\n",
                     bundle_local_id,
                     bundle.metadata.received_via_cla,
                     lora_sent ? "Y" : "N",
                     lora_required ? "(req)" : "",
                     lte_sent ? "Y" : "N",
                     lte_required ? "(req)" : "",
                     fully_forwarded ? "REMOVE" : "KEEP");

        if (fully_forwarded) {
            // Remove bundle from store
            if (store.remove(i)) {
                removed++;
                // Clean up TX record
                removeTxRecord(bundle_local_id);
            }
        }
    }

    Serial.printf("BundleManager: Checked %d bundles, removed %d\n", checked, removed);

    return removed;
}

void BundleManager::markBundlesSentViaCLA(size_t from_store_index, const char* cla_name) {
    // Find CLA index by name
    uint8_t cla_idx = 0xFF;
    for (uint8_t i = 0; i < cla_count; i++) {
        if (strcmp(clas[i]->getName(), cla_name) == 0) {
            cla_idx = i;
            break;
        }
    }
    if (cla_idx == 0xFF) {
        Serial.printf("BundleManager: markBundlesSentViaCLA — CLA '%s' not found\n", cla_name);
        return;
    }

    size_t n = store.count();
    for (size_t i = from_store_index; i < n; i++) {
        Bundle bundle;
        if (store.get(i, bundle)) {
            markSentViaCLA(bundle.metadata.local_id, cla_idx);
            Serial.printf("BundleManager: Bundle %lu pre-marked as sent via %s\n",
                          bundle.metadata.local_id, cla_name);
        }
    }
}

bool BundleManager::isSentViaCLA(uint32_t bundle_local_id, uint8_t cla_index) {
    BundleTransmitRecord* record = getTxRecord(bundle_local_id);
    if (record == nullptr) {
        return false;
    }

    return (record->sent_via_cla_mask & (1 << cla_index)) != 0;
}

void BundleManager::markSentViaCLA(uint32_t bundle_local_id, uint8_t cla_index) {
    BundleTransmitRecord* record = getTxRecord(bundle_local_id);
    if (record == nullptr) {
        // Create new record
        if (tx_record_count >= MAX_BUNDLE_STORE_SIZE) {
            Serial.println("BundleManager: WARNING - TX record table full");
            return;
        }

        record = &tx_records[tx_record_count];
        record->bundle_local_id = bundle_local_id;
        record->sent_via_cla_mask = 0;
        tx_record_count++;
    }

    record->sent_via_cla_mask |= (1 << cla_index);
}

BundleManager::BundleTransmitRecord* BundleManager::getTxRecord(uint32_t bundle_local_id) {
    for (uint8_t i = 0; i < tx_record_count; i++) {
        if (tx_records[i].bundle_local_id == bundle_local_id) {
            return &tx_records[i];
        }
    }
    return nullptr;
}

void BundleManager::removeTxRecord(uint32_t bundle_local_id) {
    for (uint8_t i = 0; i < tx_record_count; i++) {
        if (tx_records[i].bundle_local_id == bundle_local_id) {
            // Shift remaining records down
            for (uint8_t j = i; j < tx_record_count - 1; j++) {
                tx_records[j] = tx_records[j + 1];
            }
            tx_record_count--;
            return;
        }
    }
}

const char* BundleManager::deliverToLocalService(const Bundle& bundle) {
    uint32_t dest_service = bundle.primary.destination.service_number;

    for (uint8_t i = 0; i < service_count; i++) {
        if (services[i]->getServiceNumber() == dest_service) {
            services[i]->processBundle(bundle);
            return services[i]->getName();
        }
    }

    return nullptr;
}
