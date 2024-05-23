#include "RSSIAODVRouter.h"
#include "configuration.h"
#include "mesh-pb-constants.h"
#include "RadioInterface.h" // For RSSI access
#include "NodeDB.h"
#include <pb_decode.h>

// ... (other necessary includes for logging, packet manipulation, etc.)

RSSIAODVRouter::RSSIAODVRouter() {
    LOG_INFO("[RSSIAODV] Hello World from RSSIAODVRouter and Slotted ALOHA!\n");
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

    if (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        switch (p->decoded.portnum) {
            case meshtastic_PortNum_ROUTING_APP: { // Routing message
                // Check for correct payload type before accessing routing message
                meshtastic_Routing routing = meshtastic_Routing_init_zero; 
                pb_istream_t stream = pb_istream_from_buffer(p->decoded.payload.bytes, p->decoded.payload.size);
                bool decoded = pb_decode(&stream, meshtastic_Routing_fields, &routing);

                if (decoded) {
                    switch (routing.which_variant) {
                        case meshtastic_Routing_route_request_tag: { // RREQ
                            if (routing.route_request.route_count < 8) {
                                LOG_INFO("[RSSIAODV] Received RREQ from node %u to node %u\n", p->from, p->decoded.dest);
                                // handleRREQ();
                            } else {
                                LOG_ERROR("[RSSIAODV] Invalid RREQ: Maximum hop count exceeded\n");
                            }
                            break;
                        }

                        case meshtastic_Routing_route_reply_tag: { // RREP
                            if (routing.route_reply.route_count > 0) {
                                LOG_INFO("[RSSIAODV] Received RREP from node %u to node %u\n", p->from, p->decoded.dest);
                                // handleRREP();
                            } else {
                                LOG_ERROR("[RSSIAODV] Invalid RREP: Empty route\n");
                            }
                            break;
                        }

                        case meshtastic_Routing_error_reason_tag: {  // RERR
                            LOG_WARN("[RSSIAODV] Received RERR: %d\n", routing.error_reason);
                            // handleRERR()
                            break;

                        default:
                            if (p->decoded.portnum == meshtastic_PortNum_RSSIAODV_HELLO) {
                                HelloMessage hello;
                                if (decodeHelloMessage(p->decoded.payload.bytes, p->decoded.payload.size, &hello)) {
                                    // handleHELLO();
                                } else {
                                    LOG_ERROR("[RSSIAODV] Failed to decode Hello message.\n");
                                }
                            } else {
                                LOG_DEBUG("[RSSIAODV] Received message with portnum: %d\n", p->decoded.portnum);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
    Router::sniffReceived(p, c);
}

bool RSSIAODVRouter::decodeHelloMessage(const uint8_t *data, size_t data_length, HelloMessage *hello) {
    if (data_length < sizeof(HelloMessage)) {
        LOG_ERROR("[RSSIAODV] Invalid Hello message length: %u\n", data_length);
        return false;
    }

    // Extract fields from the byte array (assuming little-endian byte order)
    hello->sender = *(NodeNum*)(data);
    hello->rssi = *(int8_t*)(data + sizeof(NodeNum));
    // ... decode other fields in HelloMessage based on your encoding logic

    return true;
}