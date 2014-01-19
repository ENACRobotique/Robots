/*
 * messages.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 *
 *
 *      Remark concerning this file :
 *      The user MUST ONLY modify the content between the matching commented lines *** user XXXX start *** and *** user XXXX stop ***
 *      He/she MUST NOT modify anything outside these markers
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "network_cfg.h"
#include "../../../botNet/shared/message_header.h"

// bn_Address : cf SUBNET_MASK and ADDRxx_MASK in network_cfg.h

//message types
typedef enum{
    E_DEBUG,                // general debug
    E_DEBUG_SIGNALLING,     // debug receiver signalling
    E_DATA,                 // arbitrary data payload
    E_ACK_RESPONSE,         // ack response
    E_PING,
    E_TRACEROUTE_REQUEST,   // traceroute request
    E_TRACEROUTE_RESPONSE,  // traceroute response

/************************ user types start ************************/
    E_CBR_CTRL,
    E_TEST_PKT,
    E_WELL_CTRL,
/************************ user types stop ************************/

    E_TYPE_COUNT            // This one MUST be the last element of the enum
}E_TYPE;

/************************ !!! user action required start ************************/
/* WARNING : BN_TYPE_VERSION describes the version of the above enum.
 It is managed "by hand" by the user. It MUST be incremented (modulo 16) only when the order of the *already existing* elements
 of this enum is modified.
 Examples :
     New type AFTER the last one (but before E_TYPE_COUNT) : do not increment BN_TYPE_VERSION
     New type between 2 previously exinsting types : increment BN_TYPE_VERSION
     Cleaning (removing) unused type : increment BN_TYPE_VERSION
     Changing the order of the types : increment BN_TYPE_VERSION
 The recommended use is the following :
     Add every new type at the end of the list (but before E_TYPE_COUNT), do not delete old ones.
     Every month (or so, when it is needed) : remove unused types, sort logically the other types and increse BN_TYPE_VERSION.
 */
#define BN_TYPE_VERSION 0       //last modifed : 2014/01/16
/************************ user action required stop ************************/

//function returning a string corresponding to one element of the above enum. Must be managed by hand.
char *eType2str(E_TYPE elem);


/************************ user payload definition start ************************/
//user-defined payload types.
//for simple payloads ( single variable or array), this step can be skipped
//Warning : the user has to make sure that these payloads are not too big (cf BN_MAX_PDU)

typedef struct {
    uint8_t flag;
    int8_t fluxID;         // identifier of the flux (-1 means every flux)
    bn_Address dest;        // destination of the CBR
    uint32_t  period;       // minimal period between messages asked (in microsecond). May be exceeded by the actual sending.
    uint32_t number;        // amount of messages to send
    uint8_t size;           // size of the payload of the messages
}sCBRCtrl;

typedef struct {
    uint8_t flag;
    int8_t fluxID;         // identifier of the flux (-1 means every flux)
    bn_Address src;        // destination of the CBR
}sWellCtrl;

typedef struct {
    uint8_t fluxID;             // identifier of the flux
    uint8_t bullshit[BN_MAX_PDU-sizeof(uint8_t)-sizeof(sGenericHeader)];           // size of the payload of the messages
}sTestMsg;

/************************ user payload definition stop ************************/


/*
 * union defining the different possible payloads
 */
typedef union{
    uint8_t raw[BN_MAX_PDU-sizeof(sGenericHeader)];		//only used to access data/data modification in low layer
    uint8_t data[BN_MAX_PDU-sizeof(sGenericHeader)];	//arbitrary data, actual size given by the "size" field of the header
    uint8_t debug[BN_MAX_PDU-sizeof(sGenericHeader)];   //debug string, actual size given by the "size" field of the header
    sAckPayload ack;

/************************ user payload start ************************/
//the user-defined payloads from above must be added here. The simple ones can be directly added here
//Warning : the user has to make sure that these payloads are not too big (cf BN_MAX_PDU)

    sCBRCtrl CBRCtrl;
    sTestMsg testMsg;
    sWellCtrl wellCrtl;

/************************ user payload stop ************************/

}uPayload;


//final message structure
typedef struct{
    sGenericHeader header;
    uPayload payload;
}sMsg;

#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_H_ */
