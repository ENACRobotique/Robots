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

//network config
#define MYADDRX (SUBNETX | ADDRX_MAIN )
#define MYADDRI (SUBNETI_MAIN | ADDRI_MAIN_TURRET)
#define SB_INC_MSG_BUF_SIZE 4
#define ARCH_328P_ARDUINO




#define PIN_RST_XBEE 5
#define PIN_DBG_LED 13

#define PHASE_INIT_MOBILE_1 0 //in TR<<9 (ex. 45° = 45/360 tr = 0.125 tr = 64 tr<<64)
#define PHASE_INIT_MOBILE_2 0 //in tr<<9

#define ROT_PERIOD_BCAST 5000 //in ms

#define DEBUG

//#define DEBUG_PRINT

#endif /* PARAMS_H_ */
