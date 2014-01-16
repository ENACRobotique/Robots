/*
 * message_header.h
 *
 *  Created on: 1 oct. 2013
 *      Author: quentin
 */


#ifndef MESSAGE_HEADER_H_
#define MESSAGE_HEADER_H_

#include <stdint.h>

#define BN_MAX_PDU 64 //max size of a message, including header AND payload.

/* bn_Address : on 16 bytes,
 * cf SUBNET_MASK and ADDRxx_MASK in network_cfg.h
 *
 */
typedef uint16_t bn_Address;

// structure of a header
typedef struct {
    bn_Address destAddr;
    bn_Address srcAddr;
    uint8_t size : 7;           //size of the payload
    uint8_t ack  : 1;           //acknowledgement flag. If set, the message must be acknoledged (positively or negatively)
    uint8_t type;               //type of the message
    uint8_t typeVersion : 4;    //version of the type enum for the sender node. Increased at every change in this enum.
    uint8_t seqNum : 4;         //sequence number
    uint8_t checksum;           //checksum on the whole message (payload+header)
}sGenericHeader;

// ack answer types
typedef enum{
    A_ACK,
    A_NACK_BUFFER_FULL,     //probably useless, because waiting for the SB_ACK_TIMEOUT timeout should be long enough to slow down trafic
    A_NACK_BROKEN_LINK
}E_ACK_ANS;

typedef struct  __attribute__((__packed__)){
    uint8_t ans;      // type of ack
    uint8_t seqNum;     // Sequence number of acked message
    bn_Address addr;    // Destination address  of the acked message
}sAckPayload;

#endif /* MESSAGE_HEADER_H_ */

