#ifndef BP_ECHO_SERVICE_H
#define BP_ECHO_SERVICE_H

#include "BundleService.h"

// Forward declaration
class BundleManager;

/**
 * BPEcho Service (Service Number 12161)
 *
 * Echoes received bundles back to the sender.
 * Implements the bping echo protocol.
 */
class BPEchoService : public IBundleService {
public:
    BPEchoService(BundleManager* bundleMgr);

    void processBundle(const Bundle& bundle) override;
    uint32_t getServiceNumber() const override { return 12161; }
    const char* getName() const override { return "BPEcho"; }

private:
    BundleManager* manager;
};

#endif // BP_ECHO_SERVICE_H
