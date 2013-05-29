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


/* routing table entry
 * the routing of a message "msg", with destination address "destAddr", coming from the interface "interface" is the following :
 * while(we are not at the end of the routing table){
 *      if (interface==ifaceFrom){
 *          if (destAddr==dest || destAddr&SUBNETMASK==dest){
 *              send to ifaceTo;
 *              return;
 *              }
 *          }
 *      go to next routing table entry
 * }
 * perform default action
 *
 * /!\ the order of the entries is important : most specific first, less specific last
 * the last entry MUST be {0,0,0}
 *
 */


//interface identifiers
typedef enum{
    IF_XBEE,
    IF_I2C,
    IF_SELF,
    IF_UNKNOWN,
    IF_DISCARD,

    IFACE_COUNT
}E_IFACE;

typedef struct{
    uint16_t dest;
    E_IFACE ifaceFrom;
    E_IFACE ifaceTo;
}rTableEntry;

#define XBEE_MAX_SIZE 100

//message types
typedef enum{
    E_SWITCH_CHANNEL,       //switch channel message
    E_SYNC_EXPECTED_TIME,   //sync expected time (send from the turret to the receiver)
    E_SYNC_OK,              //synced
    E_PERIOD,               //period measurement
    E_MEASURE,              //laser delta-time measurement
    E_DATA,                 //generic data payload
    E_DEBUG,                //general debug
    E_DEBUG_ADDR,           //adress related debug
    E_RAW,

    E_TYPE_COUNT
}E_TYPE;

char *eType2str(E_TYPE elem);



typedef struct {
    uint16_t destAddr;
    uint16_t srcAddr;
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
    uint8_t raw[XBEE_MAX_SIZE-sizeof(sGenericHeader)];
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
