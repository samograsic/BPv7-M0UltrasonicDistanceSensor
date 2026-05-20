#include "BPEchoService.h"
#include "BundleManager.h"
#include <Arduino.h>

// Echo bundle lifetime (can be overridden by config.h)
#ifndef ECHO_BUNDLE_LIFETIME_MS
#define ECHO_BUNDLE_LIFETIME_MS 1800000  // 30 minutes
#endif

BPEchoService::BPEchoService(BundleManager* bundleMgr)
    : manager(bundleMgr) {
}

void BPEchoService::processBundle(const Bundle& bundle) {
    Serial.printf("BPEchoService: Echo request from ipn:%lu.%lu (%d bytes)\n",
                  bundle.primary.source.node_number,
                  bundle.primary.source.service_number,
                  bundle.payload.data_length);

    // Check for null or "dtn:none" source (don't echo these)
    if (bundle.primary.source.node_number == 0) {
        Serial.println("BPEchoService: Ignoring bundle from dtn:none");
        return;
    }

    // Set destination to the source EID
    EndpointID echoDestination;
    echoDestination.node_number = bundle.primary.source.node_number;
    echoDestination.service_number = bundle.primary.source.service_number;

    // Create echo bundle with same payload
    // Use ECHO_BUNDLE_LIFETIME_MS from config (default 30 minutes)
    // IMPORTANT: Set source service number to 12161 (BPEchoService) so bping recognizes it
    if (manager) {
        bool success = manager->createBundle(
            echoDestination,
            bundle.payload.data,
            bundle.payload.data_length,
            ECHO_BUNDLE_LIFETIME_MS,  // Use configured echo lifetime (30 minutes)
            12161                      // Source service number (BPEchoService)
        );

        if (success) {
            Serial.println("BPEchoService: Echo response queued");
        } else {
            Serial.println("BPEchoService: ERROR - Failed to create echo");
        }
    } else {
        Serial.println("BPEchoService: ERROR - No BundleManager");
    }
}
