/*
 * messages-beacons.h
 *
 *  Created on: 6 oct. 2014
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_LOCALIZATION_H_
#define LIB_NETWORK_CONFIG_MESSAGES_LOCALIZATION_H_

#include <stdint.h>

#include "messages-position.h"

// beacons synchronization

typedef struct __attribute__((__packed__)){
    uint32_t value;          //laser measured distance (in mm)
    uint32_t date;           //laser sensing time (in global time frame)
    uint16_t precision;      //precision of the measure
    uint16_t sureness;       //sureness of the mesure
} sMobileReportPayload;

typedef enum {
    SYNCF_BEGIN_ELECTION,
    SYNCF_MEASURES,
    SYNCF_END_MEASURES,
    SYNCF_OK
} syncFlag_wireless;

typedef struct __attribute__((__packed__)){
    uint32_t lastTurnDate;   //last turn date (in µs)
    uint32_t period;         //last measured period (instantaneous, measured at the same time as lastTurnDate, in µs)
    int16_t  index;          //index of the current
    syncFlag_wireless flag;
} sSyncPayload_wireless;

typedef enum {
    SYNC_UNSYNC,
    SYNC_OK,
    SYNC_QUESTION   // denotes a "what is your sync status ?" question.
} syncFlag_wired;

typedef struct __attribute__((__packed__)){
    syncFlag_wired flag;
} sSyncPayload_wired;

// absolute position

typedef struct __attribute__((__packed__)){
    s2DPosAtt pos;
    s2DPAUncert pos_u;
    s2DSpeed spd;

    uint32_t date; // date associated to given position, ... (in synchronized time)
} sDoAbsPos;

typedef enum{
    ABSPOS_KO,
    ABSPOS_OK
} eAbsPosStatus;

typedef struct __attribute__((__packed__)){
    eAbsPosStatus status;

    s2DPosAtt pos;
    s2DPAUncert pos_u;

    uint32_t date; // date associated to given position, ... (in synchronized time)
} sDoneAbsPos;

#endif /* LIB_NETWORK_CONFIG_MESSAGES_LOCALIZATION_H_ */
