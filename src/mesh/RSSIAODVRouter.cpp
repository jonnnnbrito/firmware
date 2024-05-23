#include "RSSIAODVRouter.h"
#include "configuration.h"
#include "mesh-pb-constants.h"
#include "RadioInterface.h" // For RSSI access
#include "NodeDB.h"
#include <pb_decode.h>

// ... (other necessary includes for logging, packet manipulation, etc.)

RSSIAODVRouter::RSSIAODVRouter() {
    LOG_INFO("[RSSIAODV] Hello World from RSSIAODVRouter!\n");
} 

// Override from Router
ErrorCode RSSIAODVRouter::send(meshtastic_MeshPacket *p) {

    wasSeenRecently(p);
    
    // Call base class send to handle packet transmission after AODV logic
    return Router::send(p);  
}

// Override from Router
bool RSSIAODVRouter::shouldFilterReceived(const meshtastic_MeshPacket *p) {
    if (wasSeenRecently(p)) { // Note: this will also add a recent packet record
        printPacket("Ignoring incoming msg, because we've already seen it", p);
        if (config.device.role != meshtastic_Config_DeviceConfig_Role_ROUTER &&
            config.device.role != meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT &&
            config.device.role != meshtastic_Config_DeviceConfig_Role_REPEATER) {
            Router::cancelSending(p->from, p->id); // cancel rebroadcast for this message *if* there was already one, unless we're a router/repeater!
        }
        return true;
    }

    return Router::shouldFilterReceived(p); // If not a duplicate, continue with Router's filtering
}

void RSSIAODVRouter::sniffReceived(const meshtastic_MeshPacket *p, const meshtastic_Routing *c) {
    // Duplicate suppression using PacketHistory
    if (wasSeenRecently(p)) {
        LOG_DEBUG("[RSSIAODV] Discarding duplicate packet from node %u\n", p->from);
        return; 
    }
/*
    // Packet identification
    if (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) { // Check if packet is decoded
        switch (p->decoded.portnum) {
            case meshtastic_PortNum_ROUTING_APP: { // Routing message
                if (p->decoded.which_payload == meshtastic_Data_payload_routing_tag) {
*/
    Router::sniffReceived(p, c);
}
