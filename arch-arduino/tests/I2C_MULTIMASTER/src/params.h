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
#define MYADDRI ADDRI_MAIN_TURRET
#define MYADDRX ADDRX_MAIN
#define SB_INC_MSG_BUF_SIZE 4
#define ARCH_328P_ARDUINO



#define PIN_DBG_LED 13

#define DEBUG


#endif /* PARAMS_H_ */
