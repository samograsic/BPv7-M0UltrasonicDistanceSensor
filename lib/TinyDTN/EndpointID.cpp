#include "EndpointID.h"

String EndpointID::toString() const {
    return String("ipn:") + String(node_number) + String(".") + String(service_number);
}

EndpointID EndpointID::parse(const String& eid_str) {
    // Expected format: "ipn:node.service"
    if (!eid_str.startsWith("ipn:")) {
        // Invalid format, return null endpoint
        return EndpointID(0, 0);
    }

    // Extract the part after "ipn:"
    String numeric_part = eid_str.substring(4);

    // Find the dot separator
    int dot_index = numeric_part.indexOf('.');
    if (dot_index == -1) {
        // No dot found, invalid format
        return EndpointID(0, 0);
    }

    // Split into node and service parts
    String node_str = numeric_part.substring(0, dot_index);
    String service_str = numeric_part.substring(dot_index + 1);

    // Parse to integers
    uint32_t node = node_str.toInt();
    uint32_t service = service_str.toInt();

    // Validate conversion (toInt() returns 0 on failure, but 0 is also valid)
    // Simple validation: check if strings are not empty
    if (node_str.length() == 0 || service_str.length() == 0) {
        return EndpointID(0, 0);
    }

    return EndpointID(node, service);
}

EndpointID EndpointID::fromIPN(uint32_t node, uint32_t service) {
    return EndpointID(node, service);
}

bool EndpointID::isNull() const {
    return (node_number == 0 && service_number == 0);
}

bool EndpointID::isValid() const {
    return !isNull();
}

bool EndpointID::operator==(const EndpointID& other) const {
    return (node_number == other.node_number) && (service_number == other.service_number);
}

bool EndpointID::operator!=(const EndpointID& other) const {
    return !(*this == other);
}

bool EndpointID::operator<(const EndpointID& other) const {
    if (node_number != other.node_number) {
        return node_number < other.node_number;
    }
    return service_number < other.service_number;
}
