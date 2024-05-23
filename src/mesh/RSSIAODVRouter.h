#pragma once

#include "PacketHistory.h"
#include "Router.h"
#include "NodeDB.h"  
#include <unordered_map>


struct RoutingEntry {
    NodeNum destination;
    NodeNum nextHop;
    int8_t rssi; // Received Signal Strength Indicator
    uint32_t sequenceNumber; // For freshness (optional)
    // ... other fields (e.g., lifetime, hop count) ...
};

class RSSIAODVRouter : public Router, protected PacketHistory {
public:
    RSSIAODVRouter();

    // Override Router methods
    virtual ErrorCode send(meshtastic_MeshPacket *p) override;
    virtual bool shouldFilterReceived(const meshtastic_MeshPacket *p) override;
    virtual void sniffReceived(const meshtastic_MeshPacket *p, const meshtastic_Routing *c) override;

    bool decodeRoutingMessage(const uint8_t *data, size_t data_length, meshtastic_Routing *routing);

protected:
    NodeDB* nodeDB;

private:

    
    std::unordered_map<NodeNum, RoutingEntry> routing_table;
    // AODV-specific methods
    void initiateRouteDiscovery(meshtastic_MeshPacket *p);
    void processRouteRequest(meshtastic_MeshPacket *p);
    void processRouteReply(meshtastic_MeshPacket *p);
    void updateRoutingTable(const meshtastic_MeshPacket *p);

    // Helper functions
    bool isRouteRequest(const meshtastic_MeshPacket *p) const;
    bool isRouteReply(const meshtastic_MeshPacket *p) const;
    int8_t getRSSI(const meshtastic_MeshPacket *p) const; 
    RoutingEntry* getRoute(NodeNum destination);
    // ... other helper functions ...
};
