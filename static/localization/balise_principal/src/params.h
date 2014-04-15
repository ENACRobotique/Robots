/*
 * params.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "messages.h"

typedef enum{
    S_CHANNEL,
    S_SYNC_ELECTION,
    S_SYNC_MEASURE,
    S_SYNC_END,    //waiting for beacon to acknowledge their change to game state
    S_GAME
} mainState;

typedef enum{   //flags/ID for devices to sync
//    D_FIX,
    D_MOBILE_1,
//    D_MOBILE_2,
//    D_SECONDARY,

    D_AMOUNT
}remoteDeviceID;

typedef enum{
    DS_OFF,
    DS_SYNCED,
    DS_GAME,
    DS_UNSYNCED,    //implies on

}eDeviceState;

typedef struct{
    int lastIndex;                  // last index send to this device (during sync)
    eDeviceState state;             // current recorded state of the device (from our sync POV)
    sMobileReportPayload lastData;  // last received data position data (that was not already handled because it was too early)
    bn_Address addr;
}sDeviceInfo;



#define ELECTION_TIME       2000000  // in µs, duration during which the beacon choose their laser interruption
#define SYNCRONIZATION_TIME 10000000 // in µs


#define PIN_RST_XBEE 5
#define PIN_DBG_LED 13

#define ROT_PERIOD_BCAST 1000 //in ms

#define DEBUG

//#define BLINK_1S    //blink every second (incompatible with other blink)
#define BLINK_1TR   //blink every turn (incompatible with other blink)


//#define DEBUG_PRINT

#endif /* PARAMS_H_ */
