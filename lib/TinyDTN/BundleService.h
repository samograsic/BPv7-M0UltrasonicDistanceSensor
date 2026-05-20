#ifndef BUNDLE_SERVICE_H
#define BUNDLE_SERVICE_H

#include "Bundle.h"

/**
 * Bundle Service Interface
 *
 * Base interface for application services that consume bundles.
 * Each service listens on a specific service number and processes
 * bundles destined for that service.
 */
class IBundleService {
public:
    virtual ~IBundleService() {}

    /**
     * Process a received bundle destined for this service
     *
     * @param bundle The bundle to process
     */
    virtual void processBundle(const Bundle& bundle) = 0;

    /**
     * Get the service number this service listens on
     *
     * @return Service number (e.g., 1 for debug, 12161 for echo)
     */
    virtual uint32_t getServiceNumber() const = 0;

    /**
     * Get the service name
     *
     * @return Human-readable service name
     */
    virtual const char* getName() const = 0;
};

#endif // BUNDLE_SERVICE_H
