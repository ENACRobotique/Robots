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
    S_GAME            // Game mode
} mainState;


#define SENDING_PERIOD 100 //in ms

//#define DEBUG
//#define DEBUG_SYNC
//#define DEBUG_SYNC_VALUES
//#define VERBOSE_SYNC
//#define DEBUG_PRINTLASER
//#define DEBUG_CALIBRATION

#define PIN_SYNC    8

#define SYNC_WIRED
//#define SYNC_WIRELESS

#if defined(SYNC_WIRED) && defined(SYNC_WIRELESS)
#error "only one sync method possible"
#endif


#endif /* PARAMS_H_ */
