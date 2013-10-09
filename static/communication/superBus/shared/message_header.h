/*
 * message_header.h
 *
 *  Created on: 1 oct. 2013
 *      Author: quentin
 */

#ifndef MESSAGE_HEADER_H_
#define MESSAGE_HEADER_H_

/* sb_Address : on 16 bytes,
 * cf SUBNET_MASK and ADDRxx_MASK in network_cfg.h
 *
 */
typedef uint16_t sb_Address;


typedef struct {
    sb_Address destAddr;
    sb_Address srcAddr;
    uint8_t size;       //size of the payload
    uint8_t type;       //type of the message
    uint8_t checksumHead;
    uint8_t checksumPload;
}sGenericHeader;



#endif /* MESSAGE_HEADER_H_ */

