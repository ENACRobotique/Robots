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
#define MYADDRU 0

#define SB_INC_MSG_BUF_SIZE 4
#define SB_WAIT_SND_FAIL    50000
#define ARCH_328P_ARDUINO
#define ARCH_LITTLE_ENDIAN
#define XBEE_WAITFRAME_TIMEOUT 10
#define XBEE_READBYTE_TIMEOUT 10000

#define SYNC_TOL 8 //in µs. max desync tolerated


#define DEBUG

#endif /* PARAMS_H_ */
