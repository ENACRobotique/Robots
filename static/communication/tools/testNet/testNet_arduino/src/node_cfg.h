/*
 * node_cfg.h
 *
 *  Created on: 1 oct. 2013
 *      Author: quentin
 */

#ifndef NODE_CFG_H_
#define NODE_CFG_H_

#include "network_cfg.h"

#define MYADDRX 0
#define MYADDRI ADDRI_ARDUINO
#define MYADDRU ADDRU1_ARDUINO
#define MYADDRD 0
#define MYADDR (MYADDRX?:MYADDRI?:MYADDRU?:MYADDRD)

#define MYROLE 0
// MYROLE must be equal to role_get_role(MYADDR)

#define BN_INC_MSG_BUF_SIZE 8

#define BN_WAIT_XBEE_SND_FAIL    50000
#define BN_MAX_RETRIES          2
#define BN_ACK_TIMEOUT          1000    //in ms

#define UART_WAITFRAME_TIMEOUT 10
#define UART_READBYTE_TIMEOUT 100000

#undef ADDR_DEBUG_DFLT
#define ADDR_DEBUG_DFLT 0


#endif /* NODE_CFG_H_ */
