#ifndef ENDPOINT_ID_H
#define ENDPOINT_ID_H

#include <Arduino.h>

/**
 * EndpointID - IPN (ipn:) scheme only (RFC 6260 CBHE)
 *
 * Format: ipn:node.service
 * Example: ipn:1.0 (node 1, admin service)
 *          ipn:1.1 (node 1, beacon service)
 *          ipn:0.0 (null endpoint / broadcast)
 *
 * This implementation ONLY supports IPN addressing.
 * No support for dtn:// URIs as per project requirements.
 */
struct EndpointID {
    uint32_t node_number;      // IPN node number
    uint32_t service_number;   // IPN service number (0 for admin)

    // Constructors
    EndpointID() : node_number(0), service_number(0) {}
    EndpointID(uint32_t node, uint32_t service)
        : node_number(node), service_number(service) {}

    /**
     * Convert to string representation
     * @return String in format "ipn:node.service"
     */
    String toString() const;

    /**
     * Parse IPN string to EndpointID
     * @param eid_str String in format "ipn:node.service"
     * @return EndpointID instance (returns ipn:0.0 if parsing fails)
     */
    static EndpointID parse(const String& eid_str);

    /**
     * Create EndpointID from IPN components
     * @param node Node number
     * @param service Service number
     * @return EndpointID instance
     */
    static EndpointID fromIPN(uint32_t node, uint32_t service);

    /**
     * Check if this is a null endpoint (ipn:0.0)
     * @return true if node and service are both 0
     */
    bool isNull() const;

    /**
     * Check if this is a valid endpoint
     * @return true if not null
     */
    bool isValid() const;

    /**
     * Equality operator
     */
    bool operator==(const EndpointID& other) const;

    /**
     * Inequality operator
     */
    bool operator!=(const EndpointID& other) const;

    /**
     * Less-than operator (for sorting/maps)
     */
    bool operator<(const EndpointID& other) const;
};

// Common endpoint constants
namespace Endpoints {
    const EndpointID NULL_ENDPOINT(0, 0);           // ipn:0.0 (broadcast/null)
    const EndpointID ADMIN_ENDPOINT(1, 0);          // ipn:1.0 (admin)
    const EndpointID BEACON_ENDPOINT(1, 1);         // ipn:1.1 (beacon service)
    const EndpointID SENSOR_ENDPOINT(1, 2);         // ipn:1.2 (sensor data service)
}

#endif // ENDPOINT_ID_H
