/*
 * messages-beacons.h
 *
 *  Created on: 6 oct. 2014
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_LOCALIZATION_H_
#define LIB_NETWORK_CONFIG_MESSAGES_LOCALIZATION_H_

#include <stdint.h>

// beacons synchronization

typedef struct __attribute__((__packed__)){
    uint32_t value;          //laser measured distance (in mm)
    uint32_t date;           //laser sensing time
    uint16_t precision;      //precision of the measure
    uint16_t sureness;       //sureness of the mesure
} sMobileReportPayload;

typedef enum {
    SYNCF_BEGIN_ELECTION,
    SYNCF_MEASURES,
    SYNCF_END_MEASURES,
    SYNCF_OK
} syncFlag;

typedef struct __attribute__((__packed__)){
    uint32_t lastTurnDate;   //last turn date (in µs)
    uint32_t period;         //last measured period (instantaneous, measured at the same time as lastTurnDate, in µs)
    int16_t  index;          //index of the current
    syncFlag flag;
} sSyncPayload;

#endif /* LIB_NETWORK_CONFIG_MESSAGES_LOCALIZATION_H_ */
