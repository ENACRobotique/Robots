/*
 * node_cfg.h
 *
 *  Created on: 1 oct. 2013
 *      Author: quentin
 */

#ifndef NODE_CFG_H_
#define NODE_CFG_H_

#include "network_cfg.h"

#define MYADDRX ADDRX_BEACON_2
#define MYADDRI 0
#define MYADDRU 0
#define MYADDRD 0
#define MYADDR (MYADDRX?:MYADDRI?:MYADDRU?:MYADDRD)

#define MYROLE 0
// MYROLE must be equal to role_get_role(MYADDR)

#define BN_INC_MSG_BUF_SIZE 8

#define BN_WAIT_XBEE_SND_FAIL    50000
#define BN_MAX_RETRIES          2
#define BN_ACK_TIMEOUT          1000    //in ms

#define XBEE_RST_PIN 5

#define UART_WAITFRAME_TIMEOUT 10
#define UART_READBYTE_TIMEOUT 10000

#undef ADDR_DEBUG_DFLT
#define ADDR_DEBUG_DFLT 0


#endif /* NODE_CFG_H_ */
