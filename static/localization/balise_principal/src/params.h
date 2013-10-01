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
    CHANNEL,
    SYNC,
    GAME
} mainState;



#define XBEE_WAITFRAME_TIMEOUT 10
#define XBEE_READBYTE_TIMEOUT 10000




#define PIN_RST_XBEE 5
#define PIN_DBG_LED 13

#define PHASE_INIT_MOBILE_1 0 //in TR<<9 (ex. 45° = 45/360 tr = 0.125 tr = 64 tr<<64)
#define PHASE_INIT_MOBILE_2 0 //in tr<<9

#define ROT_PERIOD_BCAST 1000 //in ms

#define MAX_ROUTINE_CALL 50 //
#define MAX_ROUTINE_TIME 5  //in ms

#define DEBUG

//#define BLINK_1S    //blink every second (incompatible with every turn)
#define BLINK_1TR   //blink every turn (incompatible with every second)

//#define DEBUG_PRINT

#endif /* PARAMS_H_ */
