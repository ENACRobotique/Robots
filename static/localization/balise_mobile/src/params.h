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


//#define DEBUG
#define DEBUG_SYNC

#if MYADDRX==ADDRX_MOBILE_1     // the beacon with a "1" written in red on the copper side of the PCB
    #define HARDUPDATEPERIOD  129870    // 1/abs(delta) or O if disabled. delta is the first order drift between the turret and the considered beacon)
    #define HARDUPDATESIGN    -1        // sgn(delta)
#endif

#endif /* PARAMS_H_ */
