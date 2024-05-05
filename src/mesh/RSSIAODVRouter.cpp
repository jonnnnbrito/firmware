#include "RSSIAODVRouter.h" 
#include <mesh-pb-constants.h> 
#include <unordered_map> 
#include <RadioLib.h>
#include "FloodingRouter.h"
#include "Router.h"
#include <modules/SX126x/SX126x.h>
#include <cstring>
#include <iterator>
#include <optional> 
#include "NodeDB.h"
#include "MeshTypes.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "ReliableRouter.h"
#include "meshtastic/portnums.pb.h"

#define RSSI_BEACON_PORT 300


// Global Variables
std::unordered_set<uint32_t> seen_rreq_ids;
std::unordered_map<NodeNum, meshtastic_MeshPacket> packetStore; 
std::unordered_map<NodeNum, uint32_t> rreq_sequence_numbers; 

const unsigned long BEACON_INTERVAL = 10000; // 10 seconds in milliseconds

RSSIAODVRouter::RSSIAODVRouter(Router& router) :
    router(router) {}

// RSSIAODVRouter Class Implementation
NodeNum RSSIAODVRouter::getNodeNum() {
    return nodeDB->getNodeNum(); // Assuming you have access to a nodeDB object
}

void RSSIAODVRouter::updateNeighborRSSI(NodeNum neighbor_id, int8_t neighbor_rssi, const meshtastic_MeshPacket *p) {
    uint32_t now = millis(); // Or a more accurate timestamp 

    // 1. Packet Validation
    if (p == nullptr) { 
        LOG_ERROR("Null packet pointer in updateNeighborRSSI");
        return; 
    }    

    // Check if neighbor exists in the table
    auto it = neighbor_rssi_table.find(neighbor_id);
    if (it != neighbor_rssi_table.end()) {
        // Entry exists - update RSSI and timestamp
        it->second.rssi = neighbor_rssi;
        it->second.last_updated_timestamp = now;
    } else {
        // New entry
        NeighborRSSI entry;
        entry.node_id = neighbor_id;
        entry.rssi = neighbor_rssi;
        entry.last_updated_timestamp = now;
        entry.hop_count = 1; // Assuming direct neighbor
        // ... (Set sequence number if applicable)

        neighbor_rssi_table[neighbor_id] = entry;
    }


    // Expiration Logic (Example):
    const uint32_t EXPIRATION_TIMEOUT = 5000; // 5 seconds
    for (auto it = neighbor_rssi_table.begin(); it != neighbor_rssi_table.end(); /* */) {
        if (now - it->second.last_updated_timestamp > EXPIRATION_TIMEOUT) {
            it = neighbor_rssi_table.erase(it); // Remove expired entry
        } else {
            ++it;
        }
    }

    // ***Integration Point***
    // Integration of handleNewPacket (now the 'packet' is defined)
    handleNewPacket(*p);     
}

bool RSSIAODVRouter::shouldFilterReceived(const meshtastic_MeshPacket *p) {
    if (p->decoded.portnum == RSSI_BEACON_PORT) {
        // Handle RSSI beacon packets
        handleRSSIBeacon(p);
        return true; // Filter out RSSI beacons 
    } else if (p->decoded.portnum == AODV_PORT) {
        if (p->hop_limit > 0) { 
            if (p->decoded.payload.size >= sizeof(uint8_t)) { 
                uint8_t packet_type = p->decoded.payload.bytes[0]; 
                if (packet_type == 1) { // RREQ
                    RREQPacket rreq = deserializeRREQ(p); 
                    if (!isRREQSeen(rreq.broadcast_id)) { 
                         addRREQToSeenList(rreq.broadcast_id); 
                         handleRREQ(p); // Your existing function to handle RREQs further
                    }
                } else if (packet_type == 2) { 
                    handleRREP(p); // Handle RREPs 
                } else {
                    // ... Handle other AODV packet types ...
                }
            } else {
                LOG_ERROR("AODV packet is too small\n"); 
            }
        } 
        return true; // Filter out AODV packets
    } else {
        // Fallback to flooding-based logic for other packets
        return floodingRouter.shouldFilterReceived(p); // Use your instance
    }
}

uint32_t extractUint32(const uint8_t* bytes, size_t offset) {
    return (uint32_t(bytes[offset]) << 24) |
           (uint32_t(bytes[offset + 1]) << 16) | 
           (uint32_t(bytes[offset + 2]) << 8) | 
           uint32_t(bytes[offset + 3]); 
}

// Helper function to extract an int8_t
int8_t extractInt8(const uint8_t* bytes, size_t offset) {
    return int8_t(bytes[offset]);
}

uint8_t extractUint8(const uint8_t* bytes, size_t offset) {
    return bytes[offset]; 
}




//*************************************************************************
// RREQ and RREP Handling Functions
//*************************************************************************  

void RSSIAODVRouter::handleRREQ(const meshtastic_MeshPacket *p) {
    // Create a mutable copy of the packet
    meshtastic_MeshPacket p_copy = *p; 

    // Modify hop_limit in the copy
    p_copy.hop_limit--;
    
    // 1. Extract RREQ fields from the packet
    RREQPacket rreq = deserializeRREQ(p); // You'll need to implement this deserialization

    auto seq_entry = rreq_sequence_numbers.find(rreq.source_node);
    if (seq_entry != rreq_sequence_numbers.end() && rreq.sequence_number <= seq_entry->second) {
        // Discard RREQ with an old or duplicate sequence number 
        return; 
    } else {
        // Update stored sequence number
        rreq_sequence_numbers[rreq.source_node] = rreq.sequence_number;
    }

    // 2. Check if this node is the destination
    if (rreq.destination_node == getNodeNum()) {
        generateRREP(rreq.source_node, rreq.route_request_id); // Function to send an RREP
        return;
    }

    // 3. Look up the destination in the routing table
    auto route_entry = findRoute(rreq.destination_node); 
    if (route_entry != nullptr) { 
        // Update if new route's RSSI is better or hop count is lower
        if (rreq.rssi > route_entry->rssi || (rreq.rssi == route_entry->rssi && rreq.hop_count < route_entry->hop_count)) {
            route_entry->next_hop = rreq.source_node; // Update next hop
            route_entry->rssi = rreq.rssi;
            route_entry->hop_count = rreq.hop_count; // Update hop count
            route_entry->expiry_time = millis() + ROUTE_EXPIRY_TIME_MS; 
        }
        generateRREP(rreq.source_node, rreq.route_request_id); 
        return;
    }  

    // 4. If no route exists and the RREQ is 'fresh':  
    if (!isRREQSeen(rreq.broadcast_id)) {
        addRREQToSeenList(rreq.broadcast_id); 

        // Update RSSI, increment hop count
        rreq.rssi = getRSSI(); 
        rreq.hop_count++; 

        // Consider RSSI thresholds before rebroadcasting
        if (p_copy.hop_limit > 0 && rreq.rssi >= RSSI_THRESHOLD) { 
            p_copy.hop_limit--; 
            rebroadcastRREQ(rreq); 
        }  
    }

    const uint32_t EXPIRATION_TIMEOUT = 5000; // 5 seconds (adjust as needed)
    for (auto it = routing_table.begin(); it != routing_table.end(); /* */) {
        if (millis() - it->second.expiry_time > EXPIRATION_TIMEOUT) {
            it = routing_table.erase(it); // Remove expired entry
        } else {
            ++it;
        }
    }

    // Integration Point 
    handleNewPacket(*p); 
}

void RSSIAODVRouter::handleRREP(const meshtastic_MeshPacket *p) {
    // 1. Extract RREP fields from the packet: 
    RREPPacket rrep = deserializeRREP(p); 

    // 2. Update your routing table with the route information:
    updateRouteTable(rrep, false); 

    // 3. Forward the RREP towards the original source (if necessary):
    if (rrep.destination_node != getNodeNum()) { 
        // Look up the next hop in your routing table:
        NodeNum next_hop = lookupNextHop(rrep.destination_node); 
        if (next_hop != 0) { // 0 might indicate an invalid next hop
            if (p->hop_limit > 0) {
                meshtastic_MeshPacket p_copy = *p;  // Create a copy of the packet
                p_copy.hop_limit--; 
                forwardRREP(rrep, next_hop); // Forward the copy
            } 
        }
    }

    // Integration Point (Optional):
    handleNewPacket(*p); 
}




//*************************************************************************
// Serialization and Deserialization Functions
//*************************************************************************

RREQPacket deserializeRREQ(const meshtastic_MeshPacket *p) {
    
    RREQPacket defaultRREQ; // Initialized with default values

    // Initialize individual members with sensible defaults:
    defaultRREQ.source_node = 0; // Assuming 0 might indicate an invalid source
    defaultRREQ.destination_node = 0; // Assuming 0 might indicate an invalid destination
    defaultRREQ.broadcast_id = 0; 
    defaultRREQ.route_request_id = 0; 
    defaultRREQ.hop_count = 0; // Or a value indicating an 'unknown' hop count
    defaultRREQ.rssi = -120; // Or a very low RSSI to indicate an invalid value 
    defaultRREQ.packet_type = 0; // Assuming 0 is not a valid packet type
    defaultRREQ.sequence_number = 0; 
    
    // Packet Size Check
    if (p->decoded.payload.size < sizeof(RREQPacket)) { 
        LOG_ERROR("RREQ packet is too short for deserialization");
        return defaultRREQ; 
    } 

    RREQPacket rreq;
    const uint8_t* payload = p->decoded.payload.bytes; 
    size_t offset = 0; 

    memcpy(&rreq.source_node, payload + offset, sizeof(rreq.source_node));
    offset += sizeof(rreq.source_node);

    memcpy(&rreq.destination_node, payload + offset, sizeof(rreq.destination_node));
    offset += sizeof(rreq.destination_node);

    memcpy(&rreq.broadcast_id, payload + offset, sizeof(rreq.broadcast_id));
    offset += sizeof(rreq.broadcast_id);

    memcpy(&rreq.route_request_id, payload + offset, sizeof(rreq.route_request_id));
    offset += sizeof(rreq.route_request_id);

    memcpy(&rreq.hop_count, payload + offset, sizeof(rreq.hop_count));
    offset += sizeof(rreq.hop_count);

    memcpy(&rreq.rssi, payload + offset, sizeof(rreq.rssi));
    offset += sizeof(rreq.rssi);

    memcpy(&rreq.packet_type, payload + offset, sizeof(rreq.packet_type));
    offset += sizeof(rreq.packet_type);       

    memcpy(&rreq.sequence_number, payload + offset, sizeof(rreq.sequence_number));
    offset += sizeof(rreq.sequence_number);

    return rreq;
}

RREPPacket deserializeRREP(const meshtastic_MeshPacket *p) {
    RREPPacket defaultRREP; // Create a default RREPPacket

    // Initialize individual members with sensible defaults:
    defaultRREP.source_node = 0; // Assuming 0 might indicate an invalid source node
    defaultRREP.destination_node = 0; // Assuming 0 might indicate an invalid destination node
    defaultRREP.route_request_id = 0;  
    defaultRREP.hop_count = UINT8_MAX; // Or a value indicating an unreachable destination 
    defaultRREP.rssi = -120; // Or a very low RSSI to indicate an invalid value 
    defaultRREP.packet_type = 0; // Assuming 0 is not a valid AODV packet type
    defaultRREP.sequence_number = 0;
    
    // Packet Size Check
    if (p->decoded.payload.size < sizeof(RREPPacket)) { 
        LOG_ERROR("RREP packet is too short for deserialization");
        return defaultRREP; 
    }

    RREPPacket rrep;
    const uint8_t* payload = p->decoded.payload.bytes; 
    size_t offset = 0; 

    rrep.source_node = extractUint32(payload, offset); 
    offset += sizeof(rrep.source_node);

    rrep.destination_node = extractUint32(payload, offset); 
    offset += sizeof(rrep.destination_node);

    rrep.route_request_id = extractUint32(payload, offset); 
    offset += sizeof(rrep.route_request_id);

    rrep.hop_count = extractInt8(payload, offset); 
    offset += sizeof(rrep.hop_count);

    rrep.rssi = extractInt8(payload, offset); 
    offset += sizeof(rrep.rssi);

    rrep.packet_type = extractUint8(payload, offset); 
    offset += sizeof(rrep.packet_type); 

    rrep.sequence_number = extractUint32(payload, offset); 
    offset += sizeof(rrep.sequence_number);
    
    return rrep;

}

uint8_t* RSSIAODVRouter::serializeRREP(const RREPPacket &rrep) {
    const int BUFFER_SIZE = sizeof(uint8_t) +      // packet_type
                            sizeof(uint32_t) +     // source_node
                            sizeof(uint32_t) +     // destination_node
                            sizeof(uint32_t) +     // route_request_id
                            sizeof(uint32_t) +     // sequence_number 
                            sizeof(int) +          // hop_count
                            sizeof(int8_t);        // rssi

    uint8_t* buffer = new uint8_t[BUFFER_SIZE];
    if (buffer == nullptr) { // Check for allocation failure
        LOG_ERROR("Memory allocation for RREP serialization failed");
        return nullptr;  // Or throw an exception if suitable
    }
    size_t offset = 0; 

    memcpy(buffer + offset, &rrep.source_node, sizeof(rrep.source_node)); 
    offset += sizeof(rrep.source_node);

    memcpy(buffer + offset, &rrep.destination_node, sizeof(rrep.destination_node)); 
    offset += sizeof(rrep.destination_node);

    memcpy(buffer + offset, &rrep.route_request_id, sizeof(rrep.route_request_id)); 
    offset += sizeof(rrep.route_request_id);

    memcpy(buffer + offset, &rrep.hop_count, sizeof(rrep.hop_count)); 
    offset += sizeof(rrep.hop_count);

    memcpy(buffer + offset, &rrep.rssi, sizeof(rrep.rssi)); 
    offset += sizeof(rrep.rssi);

    memcpy(buffer + offset, &rrep.packet_type, sizeof(rrep.packet_type));
    offset += sizeof(rrep.packet_type); 

    memcpy(buffer + offset, &rrep.sequence_number, sizeof(rrep.sequence_number));
    offset += sizeof(rrep.sequence_number);

    return buffer; 
}

uint8_t* RSSIAODVRouter::serializeRREQ(const RREQPacket &rreq) {
    const int BUFFER_SIZE = sizeof(uint32_t) + // source_node 
                            sizeof(uint32_t) + // destination_node
                            sizeof(uint32_t) + // route_request_id
                            sizeof(uint32_t) + // sequence_number 
                            sizeof(int) +      // hop_count
                            sizeof(int8_t) +   // rssi
                            sizeof(uint8_t) +  // packet_type
                            sizeof(uint32_t);  // broadcast_id

    uint8_t* buffer = new uint8_t[BUFFER_SIZE];
    if (buffer == nullptr) { 
        LOG_ERROR("Memory allocation for RREQ serialization failed");
        return nullptr;
    }

    size_t offset = 0; 

    memcpy(buffer + offset, &rreq.source_node, sizeof(rreq.source_node));
    offset += sizeof(rreq.source_node);

    memcpy(buffer + offset, &rreq.destination_node, sizeof(rreq.destination_node));
    offset += sizeof(rreq.destination_node);

    memcpy(buffer + offset, &rreq.broadcast_id, sizeof(rreq.broadcast_id));
    offset += sizeof(rreq.broadcast_id);

    memcpy(buffer + offset, &rreq.route_request_id, sizeof(rreq.route_request_id));
    offset += sizeof(rreq.route_request_id);

    memcpy(buffer + offset, &rreq.hop_count, sizeof(rreq.hop_count));
    offset += sizeof(rreq.hop_count);

    memcpy(buffer + offset, &rreq.rssi, sizeof(rreq.rssi));
    offset += sizeof(rreq.rssi);

    memcpy(buffer + offset, &rreq.packet_type, sizeof(rreq.packet_type));
    offset += sizeof(rreq.packet_type); 

    memcpy(buffer + offset, &rreq.sequence_number, sizeof(rreq.sequence_number));
    offset += sizeof(rreq.sequence_number);

    return buffer; 
}








void RSSIAODVRouter::generateRREP(NodeNum destination_node, uint32_t route_request_id) {
    // 1. Create an RREPPacket 
    RREPPacket rrep;

    // 2. Populate the RREP fields
    rrep.packet_type = 2; // Indicating RREP 
    rrep.source_node = getNodeNum(); // Node generating the RREP
    rrep.destination_node = destination_node; 
    rrep.route_request_id = route_request_id;
    rrep.hop_count = lookupHopCount(destination_node);  // Or set it based on your routing table 
    rrep.rssi = getRSSIForNeighbor(destination_node); ; // Assuming you have this function

    // 3. Serialize the RREP packet
    uint8_t* packet_bytes = serializeRREP(rrep); // You'll need to implement this serialization function
    size_t packet_size = 25; // Determine the size of the serialized packet

    // 4. Create a new meshtastic_MeshPacket
    meshtastic_MeshPacket packet; 

    // 5. Populate packet fields (destination, port, etc.)
    packet.to = destination_node; 
    packet.decoded.portnum = AODV_PORT; 
    memcpy(packet.decoded.payload.bytes, packet_bytes, packet_size);
    packet.decoded.payload.size = packet_size;

    // 6. Send the RREP packet
    router.send(&packet); 
}

int8_t RSSIAODVRouter::getRSSI() {
    meshtastic_MeshPacket packet = getRecentPacketFromDatabase(); 
    return packet.rx_rssi; // Replace 'rx_rssi' with the actual field name
}

meshtastic_MeshPacket RSSIAODVRouter::getRecentPacketFromDatabase() {
    // 1. Check neighbor_rssi_table for the most recent packet
    if (!neighbor_rssi_table.empty()) {
        std::unordered_map<NodeNum, NeighborRSSI>::iterator mostRecentEntry = std::max_element(
                neighbor_rssi_table.begin(), neighbor_rssi_table.end(), 
                [](const std::pair<NodeNum, NeighborRSSI> &a, const std::pair<NodeNum, NeighborRSSI> &b) { 
                    return a.second.last_updated_timestamp < b.second.last_updated_timestamp; 
                }); 

        // Extract source_node from the map's key
        NodeNum source_node = mostRecentEntry->first; 

        // Attempt to fetch from your database 
        meshtastic_MeshPacket* packetPtr = fetchPacketFromDatabase(source_node); 
        if (packetPtr != nullptr) {
            return *packetPtr; // Success! Return the fetched packet
        } 
    }

    // 2. Fallback: Return an empty packet
    return {}; 
}

meshtastic_MeshPacket* RSSIAODVRouter::fetchPacketFromDatabase(NodeNum source_node) {
    auto it = packetStore.find(source_node);
    if (it != packetStore.end()) {
        return &it->second; // Return a pointer to the packet
    } else {
        return nullptr;  // No packet found
    }
}

int8_t RSSIAODVRouter::getRSSIForNeighbor(NodeNum neighbor_id) {
    auto it = neighbor_rssi_table.find(neighbor_id);
    if (it != neighbor_rssi_table.end()) {
        return it->second.rssi;  // Return the stored RSSI
    } else {
        return -120; // Or some default value to indicate neighbor not found
    }
}




//*************************************************************************
// Routing Table Functions
//*************************************************************************

NodeNum RSSIAODVRouter::lookupNextHop(NodeNum destination_node) {
    auto it = routing_table.find(destination_node);
    if (it != routing_table.end()) {
        return it->second.next_hop; // Return the 'next_hop' member of RouteEntry
    } else {
        return 0; // Or another value to signal "no route found"
    }
}

RouteEntry* RSSIAODVRouter::findRoute(NodeNum destination_node) {
    auto it = routing_table.find(destination_node);
    if (it != routing_table.end()) {
        // Check if the route has expired
        if (millis() - it->second.expiry_time > ROUTE_EXPIRY_TIME_MS) {
            routing_table.erase(it); // Remove expired route
            return nullptr;          // Indicate no route found
        } else {
            return &it->second; 
        }
    } else {
        return nullptr; // Route not found
    }
}

void RSSIAODVRouter::addOrUpdateRoute(const RREPPacket &rrep) {
    // 1. Extract necessary information from the RREP:
    NodeNum destination_node = rrep.destination_node;
    NodeNum next_hop = rrep.source_node; // Assuming the RREP's source is the next hop
    int8_t rssi = rrep.rssi;
    uint16_t hop_count = rrep.hop_count; 

    // 2. Attempt to find an existing route to the destination
    RouteEntry* route_entry = findRoute(destination_node);

    // 3. If a route exists:
    if (route_entry != nullptr) {
        // Update only if the new RSSI is better 
        if (rssi > route_entry->rssi || (rssi == route_entry->rssi && hop_count < route_entry->hop_count)) {
            route_entry->next_hop = next_hop;
            route_entry->rssi = rssi;
            // Keep the existing hop_count 
            route_entry->expiry_time = millis() + ROUTE_EXPIRY_TIME_MS;  // Update expiry time if needed
            // ... update any other relevant metrics ...
        }
    } else { 
        // 4. If no route exists:
        // a. Create a new RouteEntry
        RouteEntry new_entry;
        new_entry.destination = destination_node;
        new_entry.next_hop = next_hop;
        new_entry.rssi = rssi;
        new_entry.hop_count = rrep.hop_count + 1; // Increment for new routes

        // b. Add the new route entry to your routing table
        routing_table[destination_node] = new_entry;
    }
}

uint16_t RSSIAODVRouter::lookupHopCount(NodeNum destination_node) {
    RouteEntry* route_entry = findRoute(destination_node);
    if (route_entry != nullptr) {
        return route_entry->hop_count;
    } else {
        return UINT16_MAX; // A special value to signal "route not found" 
    }
}

void RSSIAODVRouter::updateRouteTable(const RREPPacket &rrep, bool invalidate) {
    NodeNum destination_node = rrep.destination_node;
    NodeNum next_hop = rrep.source_node; 
    int8_t rssi = rrep.rssi;
    uint16_t hop_count = rrep.hop_count;

    auto route_entry = findRoute(destination_node);   // Use your existing findRoute

    if (route_entry != nullptr) {
        // Route exists: Update Logic
        if (rssi > route_entry->rssi || 
            (rssi == route_entry->rssi && hop_count < route_entry->hop_count) ||
            invalidate) 
         {
            route_entry->next_hop = next_hop;
            route_entry->rssi = rssi;
            route_entry->hop_count = invalidate ? UINT16_MAX : hop_count; // Invalidate if needed
            route_entry->expiry_time = millis() + ROUTE_EXPIRY_TIME_MS;
        }
    } else {
        // Route doesn't exist: Add new route
        RouteEntry new_entry;
        new_entry.destination = destination_node;
        new_entry.next_hop = next_hop;
        new_entry.rssi = rssi;
        new_entry.hop_count = hop_count + 1; 
        new_entry.expiry_time = millis() + ROUTE_EXPIRY_TIME_MS; 
        routing_table[destination_node] = new_entry;
    }
}



//*************************************************************************
// RREQ Handling Functions
//*************************************************************************

void RSSIAODVRouter::rebroadcastRREQ(RREQPacket &rreq) {
    // 1. Check Rebroadcast Conditions
    if (rreq.hop_count > 0 && rreq.rssi >= RSSI_THRESHOLD && !isRREQSeen(rreq.broadcast_id)) { 
        addRREQToSeenList(rreq.broadcast_id); // Add to seen list before rebroadcasting

        // 2. Decrement Hop Count
        rreq.hop_count--; 

        // 3. Serialize the RREQ Packet
        uint8_t* packet_bytes = serializeRREQ(rreq);  
        size_t packet_size = 25; // Determine the size of the serialized packet

        // 4. Create a meshtastic_MeshPacket 
        meshtastic_MeshPacket packet;

        // 5. Set Broadcast Destination (*)
        packet.to = rreq.destination_node; // Assuming the preferred destination is already in the RREQ

        // 6. Set Port and Payload
        packet.decoded.portnum = AODV_PORT;
        memcpy(packet.decoded.payload.bytes, packet_bytes, packet_size);
        packet.decoded.payload.size = packet_size;

        // 7. Broadcast the Packet
        router.send(&packet); 
    }
}




//*************************************************************************
// RREP Forwarding Functions
//*************************************************************************

// Inside your RSSIAODVRouter class:
void RSSIAODVRouter::forwardRREP(const RREPPacket &rrep, NodeNum next_hop) {
    // 1. Modify RREP (decrement hop count)
    RREPPacket modifiedRrep = rrep; // Create a copy
    if (modifiedRrep.hop_count > 0) { 
        modifiedRrep.hop_count--;
    }

    // 2. Serialize
    uint8_t* packet_bytes = serializeRREP(modifiedRrep);
    size_t packet_size = 25; // Or calculate the actual size

    // 3. Create a Meshtastic packet
    meshtastic_MeshPacket packet;
    packet.to = next_hop;
    packet.decoded.portnum = AODV_PORT;
    memcpy(packet.decoded.payload.bytes, packet_bytes, packet_size);
    packet.decoded.payload.size = packet_size;

    // 4. Transmit
    router.send(&packet); 

    // Note: Consider deleting packet_bytes since you created it dynamically 
}




//*************************************************************************
// RREP Forwarding Functions
//*************************************************************************

void RSSIAODVRouter::handleNewPacket(meshtastic_MeshPacket packet) {
    NodeNum sourceNode = packet.from; // Assuming you can get the source node 
    packetStore[sourceNode] = packet; // Store the packet
}




void RSSIAODVRouter::sendRSSIBeacon() {
    // 1. Construct the RSSI Beacon Packet
    meshtastic_MeshPacket packet;

    // Set Destination (Broadcast)
    packet.to = NODENUM_BROADCAST; 

    // Set Port Number
    packet.decoded.portnum = (meshtastic_PortNum) RSSI_BEACON_PORT;

    // Encode RSSI 
    int8_t my_rssi = getRSSI(); 
    uint8_t *payload = serializeRSSI(my_rssi); // Keep your existing serialization for simplicity
    size_t payload_size = sizeof(my_rssi);

    // Populate Packet Payload
    memcpy(packet.decoded.payload.bytes, payload, payload_size);
    packet.decoded.payload.size = payload_size;

    // 2. Send the Beacon Packet
    router.send(&packet); 

    // Schedule the next beacon using millis()
    unsigned long now = millis();
    this->nextBeaconTime = now + BEACON_INTERVAL; // Use a constant for readability
}

// Inside your main Meshtastic loop (or a periodically called function): 
void RSSIAODVRouter::manageBeacons() {
    unsigned long now = millis();
    if (now >= nextBeaconTime) {
        sendRSSIBeacon(); 
    }
}

// Simple serialization for this example:
uint8_t* RSSIAODVRouter::serializeRSSI(int8_t rssi) {
    static uint8_t buffer[1];
    buffer[0] = rssi; 
    return buffer;
}

void RSSIAODVRouter::handleRSSIBeacon(const meshtastic_MeshPacket *p) {
    // 1. Extract RSSI from the beacon packet
    int8_t neighbor_rssi = deserializeRSSI(p->decoded.payload.bytes);

    // 2. Extract the source node from the beacon
    NodeNum neighbor_id = p->from;

    // 3. Update Neighbor RSSI
    updateNeighborRSSI(neighbor_id, neighbor_rssi, p); 
}

// Simple deserialization for this example:
int8_t RSSIAODVRouter::deserializeRSSI(const uint8_t *payload) {
    return (int8_t)payload[0];
}