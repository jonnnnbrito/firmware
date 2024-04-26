#ifndef RSSIAODVROUTER_H
#define RSSIAODVROUTER_H

#include <mesh-pb-constants.h> 
#include <unordered_map>
#include "MeshTypes.h"
#include <RadioLib.h>

// Port number definitions
const uint16_t RSSI_BEACON_PORT = 300; // Or your preferred port number
const uint16_t AODV_PORT_NUM     = 400; // Or your preferred port number
const uint32_t ROUTE_EXPIRY_TIME_MS = 30000;
const int8_t RSSI_THRESHOLD = -80;

std::unordered_set<uint32_t> seen_rreq_ids;
std::unordered_map<NodeNum, meshtastic_MeshPacket> packetStore; 
std::unordered_map<NodeNum, uint32_t> rreq_sequence_numbers; 

// Data structure for Neighbor RSSI
struct NeighborRSSI {
    NodeNum node_id;
    int8_t rssi;
    uint32_t last_updated_timestamp; 
    uint16_t hop_count; 
    // Add other relevant fields like source_node, etc. 
};

struct RouteEntry {
   NodeNum destination; 
   NodeNum next_hop;  
   int hop_count;         
   int8_t rssi; // RSSI of the link to the next hop
   uint32_t expiry_time; 
};

struct RREQPacket {
    NodeNum source_node;   
    NodeNum destination_node;   
    uint32_t broadcast_id;       
    uint32_t route_request_id;   
    int hop_count;              
    int8_t rssi; 
    uint8_t packet_type;
    uint32_t sequence_number;
};

struct RREPPacket {
    NodeNum source_node;    
    NodeNum destination_node; 
    uint32_t route_request_id;  
    int hop_count;             
    int8_t rssi;
    uint8_t packet_type; 
    uint32_t sequence_number; 
};

struct DataPacket {
   NodeNum destination_node;
   NodeNum source_node; 
   uint32_t packet_id; 
   uint8_t payload_type; 
   uint8_t* payload; 
   uint16_t payload_size; 
};

RREQPacket deserializeRREQ(const meshtastic_MeshPacket *p);
RREPPacket deserializeRREP(const meshtastic_MeshPacket *p);

class RSSIAODVRouter {
public:
    RSSIAODVRouter(Router& router) : router(router) {}
    FloodingRouter floodingRouter; // Or FloodingRouter* floodingRouter;

    void updateNeighborRSSI(NodeNum neighbor_id, int8_t neighbor_rssi);

    void handleRREQ(const meshtastic_MeshPacket *p);

    void handleRREP(const meshtastic_MeshPacket *p);

    NodeNum lookupNextHop(NodeNum destination_node);

    void generateRREP(NodeNum destination_node, uint32_t route_request_id);

    int8_t getRSSI();

    uint8_t *serializeRREP(const RREPPacket &rrep);

    uint8_t *serializeRREQ(const RREQPacket &rreq);

    meshtastic_MeshPacket getRecentPacketFromDatabase();

    meshtastic_MeshPacket* fetchPacketFromDatabase(NodeNum source_node); 

    int8_t getRSSIForNeighbor(NodeNum neighbor_id);

    RouteEntry *findRoute(NodeNum destination_node);

    void addOrUpdateRoute(const RREPPacket &rrep);

    uint16_t lookupHopCount(NodeNum destination_node);

    void updateRouteTable(const RREPPacket &rrep, bool invalidate);

    void updateRouteTable(const RREQPacket &rreq, bool invalidate);

    bool isRREQSeen(uint32_t broadcast_id);

    void rebroadcastRREQ(RREQPacket &rreq);

    void forwardRREP(const RREPPacket &rrep, NodeNum next_hop);

    void handleNewPacket(meshtastic_MeshPacket packet);

    RSSIAODVRouter();

    NodeNum getNodeNum();

    bool shouldFilterReceived(const meshtastic_MeshPacket *p);

    void sendRSSIBeacon(); 
    void handleRSSIBeacon(const meshtastic_MeshPacket *p); 

    Router& router;

    RSSIAODVRouter(Router& router) : router(router) {} 


    std::unordered_set<uint32_t> seen_rreq_ids; 


    bool isRREQSeen(uint32_t broadcast_id) {
        return seen_rreq_ids.find(broadcast_id) != seen_rreq_ids.end(); 
    }

    void addRREQToSeenList(uint32_t broadcast_id) {
        seen_rreq_ids.insert(broadcast_id);
    }

    std::unordered_set<uint32_t> seen_rreq_ids;

private: 
    std::unordered_map<NodeNum, RouteEntry> routing_table;
    

    // ... You might add other AODV-related methods here as needed ...
};

#endif // RSSIAODVROUTER_H