/*
 * messages.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* sb_Address : on 16 bytes,
 * cf SUBNET_MASK and ADDRxx_MASK in network_cfg.h
 *
 */
typedef uint16_t sb_Address;


#define SB_MAX_PDU 64

//message types
typedef enum{
    E_SWITCH_CHANNEL,       //switch channel message
    E_SYNC_EXPECTED_TIME,   //sync expected time (send from the turret to the receiver)
    E_SYNC_OK,              //synced
    E_PERIOD,               //period measurement
    E_MEASURE,              //laser delta-time measurement
    E_DATA,                 //arbitrary data payload
    E_DEBUG,                //general debug

    E_TYPE_COUNT
}E_TYPE;

char *eType2str(E_TYPE elem);


typedef struct {
    sb_Address destAddr;
    sb_Address srcAddr;
    uint8_t size;       //size of the payload
    uint8_t type;       //type of the message
    uint8_t checksumHead;
    uint8_t checksumPload;
}sGenericHeader;

typedef struct __attribute__((__packed__)){
    uint32_t value;        //laser measured delta-time
    uint32_t date;           //laser sensing time
    uint16_t precision;      //precision of the measure
    uint16_t sureness;       //sureness of the mesure
} sMesPayload;

typedef struct __attribute__((__packed__)) {
    uint32_t u;
    int32_t i;
    uint8_t msg[32];
} sDebugPayload; //debug message


typedef union{
    uint8_t raw[SB_MAX_PDU-sizeof(sGenericHeader)];		//only used to access data/data modification in low layer
    uint8_t data[SB_MAX_PDU-sizeof(sGenericHeader)];	//arbitrary data, actual size given by the "size" field of the header
    uint32_t syncTime;
    uint8_t channel;
    uint32_t period;
    sMesPayload measure;
    sDebugPayload debug;

}uPayload;

typedef struct{
    sGenericHeader header;
    uPayload payload;
}sMsg;

#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_H_ */
