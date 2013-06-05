/*
 * params.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "network_cfg.h"

typedef enum{
    CHANNEL,
    SYNC,
    GAME
} mainState;

#define MYADDRX (ADDRX_MOBILE_2|SUBNETX)
#define MYADDRI 0

#define SYNC_TOL 8 //in µs. max desync tolerated

#define SB_INC_MSG_BUF_SIZE 4

#define ARCH_328P_ARDUINO

#define DEBUG

#endif /* PARAMS_H_ */
