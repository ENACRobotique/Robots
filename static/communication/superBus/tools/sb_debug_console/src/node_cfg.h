/*
 * node_cfg.h
 *
 *  Created on: 1 oct. 2013
 *      Author: quentin
 */

#ifndef NODE_CFG_H_
#define NODE_CFG_H_

#include "network_cfg.h"

#define MYADDRX ADDRX_DEBUG
#define MYADDRI 0
#define MYADDRU 0

#define SB_INC_MSG_BUF_SIZE 4
#define SB_WAIT_XBEE_SND_FAIL    5000000
#define SB_MAX_RETRIES          2
#define SB_ACK_TIMEOUT          1000    //in ms

#define ARCH_X86_LINUX
#define ARCH_LITTLE_ENDIAN
#define UART_WAITFRAME_TIMEOUT 10
#define UART_READBYTE_TIMEOUT 100000


#endif /* NODE_CFG_H_ */
