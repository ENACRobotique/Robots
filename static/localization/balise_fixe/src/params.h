/*
 * params.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "node_cfg.h"

typedef enum{
    S_BEGIN,
    S_CHANNEL,        // Initial channel selection
    S_SYNC_ELECTION,  // Sync laser interruption election
    S_SYNC_MEASURES,  // Clock drift measurement
    S_SYNCED,         // Waiting until everybody is synced (message from turret tells us)
    S_GAME            // Game mode
} mainState;


#define SENDING_PERIOD 1100 //in ms

//#define DEBUG
//#define DEBUG_SYNC
//#define DEBUG_SYNC_VALUES
#if MYADDRX==ADDRX_MOBILE_1             // the beacon with a "1" written in red on the PCB
    #define HARDUPDATEPERIOD  129870    // 1/abs(delta) or O if disabled. delta is the first order drift between the turret and the considered beacon)
    #define HARDUPDATESIGN    -1        // sgn(delta)
#elif MYADDRX==ADDRX_MOBILE_2           // the beacon with a "2" written in red on the PCB
    #define HARDUPDATEPERIOD  117855    // 1/abs(delta) or O if disabled. delta is the first order drift between the turret and the considered beacon)
    #define HARDUPDATESIGN    -1        // sgn(delta)
#elif MYADDRX==ADDRX_FIX
    #define HARDUPDATEPERIOD  0         // 1/abs(delta) or O if disabled. delta is the first order drift between the turret and the considered beacon)
    #define HARDUPDATESIGN    0         // sgn(delta)
//FIXME !!!
#endif

#endif /* PARAMS_H_ */