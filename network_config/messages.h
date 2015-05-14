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

#include <stdint.h>
#include "../static/communication/botNet/shared/message_header.h"

// bn_Address : cf SUBNET_MASK and ADDRxx_MASK in network_cfg.h

#define BN_MAX_PAYLOAD (BN_MAX_PDU - sizeof(sGenericHeader))

//message types
typedef enum{
    E_DEBUG,                //  general debug
    E_ROLE_SETUP,           // @payload.roleSetup: setup addresses of some standard nodes + automatic relaying of messages between them
    E_DATA,                 //  arbitrary data payload
    E_ACK_RESPONSE,         // @payload.ack: ack response
    E_PING,
    E_TRACEROUTE_REQUEST,   //  traceroute request
    E_TRACEROUTE_RESPONSE,  //  traceroute response
    E_INTP,                 // @payload.intp: bn_intp synchronization message

/************************ user types start ************************/
    E_SYNC_DATA,            // @payload.syncWireless: sync data (send from the turret to the receiver)
    E_SYNC_STATUS,          // @payload.syncWired: sync status
    E_PERIOD,               // @payload.period period measurement
    E_MEASURE,              // @payload.mobileReport laser delta-time measurement
    E_TRAJ,                 // @payload.traj: a trajectory step
    E_ASSERV_STATS,         // @payload.asservStats: control loop statistics
    E_GOAL,                 // @payload.pos asks the robot to go to this goal (x,y)
    E_OBS_CFG,              // @payload.obsCfg: obstacle array configuration
    E_OBSS,                 // @payload.obss: obstacles update (position & status update)
    E_POS_QUERY,            // @payload.posQuery: position query
    E_SERVOS,               // @payload.servos: servo messages (posStats)
    E_IHM_STATUS,           // @payload.ihmStatus: ihm status
    E_SPEED_SETPOINT,       // @payload.speedSetPoint: speed setpoint
    E_GENERIC_POS_STATUS,   // @payload.genericPosStatus: generic position and status of an element
    E_POS_STATS,            // @payload.posStats: position statistics (packed)
    E_TRAJ_ORIENT_EL,       // @payload.trajOrientEl: complex trajectory element (position + orientation wrt time)
    E_POS_CAM,              // @payload.posCam: For position computed from CAM
    E_SYNC_QUERY,           // @payload.syncQuery: for time synchronization
    E_SYNC_RESPONSE,        // @payload.syncResponse: for time synchronization
/************************ user types stop ************************/

    E_TYPE_COUNT            // This one MUST be the last element of the enum
}E_TYPE;

/************************ !!! user action required start ************************/
/* WARNING : BN_TYPE_VERSION describes the version of the above enum.
 It is managed "by hand" by the user. It MUST be incremented (modulo 16) only when the order of the *already existing* elements
 of this enum is modified.
 Examples :
     New type AFTER the last one (but before E_TYPE_COUNT) : do not increment BN_TYPE_VERSION
     New type between 2 previously existing types : increment BN_TYPE_VERSION
     Cleaning (removing) unused type : increment BN_TYPE_VERSION
     Changing the order of the types : increment BN_TYPE_VERSION
 The recommended use is the following :
     Add every new type at the end of the list (but before E_TYPE_COUNT), do not delete old ones.
     Every month (or so, when it is needed) : remove unused types, sort logically the other types and increase BN_TYPE_VERSION.
 */
#define BN_TYPE_VERSION 0       //last modified : 2013/10/31
/************************ user action required stop ************************/

//function returning a string corresponding to one element of the above enum. Must be managed by hand.
const char *eType2str(E_TYPE elem);

/************************ user payload definition start ************************/
//user-defined payload types.
//for simple payloads ( single variable or array), this step can be skipped
//Warning : the user has to make sure that these payloads are not too big (cf BN_MAX_PDU)

#include "messages-network.h"
#include "messages-elements.h"
#include "messages-position.h"
#include "messages-locomotion.h"
#include "messages-localization.h"
#include "messages-interactions.h"
#include "messages-statuses.h"
#include "messages-image-processing.h"

/************************ user payload definition stop ************************/



/*
 * union defining the different possible payloads
 */
typedef union{
    uint8_t raw[BN_MAX_PAYLOAD];        //  only used to access data/data modification in low layer
    uint8_t data[BN_MAX_PAYLOAD];       // E_DATA (arbitrary data, actual size given by the "size" field of the header)
    uint8_t debug[BN_MAX_PAYLOAD];      // E_DEBUG (debug string, actual size given by the "size" field of the header or '\0' terminated)

// NETWORK (cf messages-network.h)
    sAckPayload ack;                    // E_ACK_RESPONSE
    sRoleSetupPayload roleSetup;        // E_ROLE_SETUP
    sINTP   intp;                       // E_INTP

/************************ user payload start ************************/
//the user-defined payloads from above must be added here. The simple ones can be directly added here
//Warning : the user has to make sure that these payloads are not too big (cf BN_MAX_PDU)

// POSITION (cf messages-position.h)
    sPosQuery posQuery;                 // E_POS_QUERY
    sPosStats posStats;                 // E_POS_STATS

// LOCOMOTION (cf messages-locomotion.h)
    sSpeedSetPoint speedSetPoint;       // E_SPEED_SETPOINT
    sTrajElRaw_t traj;                  // E_TRAJ (deprecated use trajOrientEl instead)
    sAsservStats asservStats;           // E_ASSERV_STATS
    sTrajOrientElRaw_t trajOrientEl;    // E_TRAJ_ORIENT_EL

// LOCALIZATION (cf messages-localization.h)
    sMobileReportPayload mobileReport;  // E_MEASURE
    sSyncPayload_wireless syncWireless; // E_SYNC_DATA
    sSyncPayload_wired syncWired;       // E_SYNC_STATUS
    uint8_t channel;                    // E_
    uint32_t period;                    // E_E_PERIOD

// INTERACTIONS (cf messages-interactions.h)
    sServos servos;                     // E_SERVOS
    sIhmStatus ihmStatus;               // E_IHM_STATUS

// STATUSES (cf messages-statuses.h)
    sObsConfig obsCfg;                  // E_OBS_CFG
    sObss obss;                         // E_OBSS
    sGenericPosStatus genericPosStatus; // E_GENERIC_POS_STATUS
    sSyncQuery syncQuery;               // E_SYNC_QUERY
    sSyncResponse syncResponse;         // E_SYNC_RESPONSE

// IMAGE PROCESSING (cf messages-image-processing.h)
    sPosCam posCam;                     // E_POS_CAM
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
