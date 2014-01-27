/*
 * node_cfg.h
 *
 *  Created on: 16 janv. 2014
 *      Author: quentin
 */

#ifndef NODE_CFG_H_
#define NODE_CFG_H_

#include "network_cfg.h"

#define MYADDRX 0
#define MYADDRI 0
#define MYADDRU ADDRU1_PC2

#define BN_INC_MSG_BUF_SIZE 4
#define BN_WAIT_XBEE_SND_FAIL    5000000
#define BN_MAX_RETRIES          2
#define BN_ACK_TIMEOUT          1000    //in ms

#define BN_UART_PATH "/dev/ttyACM0"

#define UART_WAITFRAME_TIMEOUT 10       //in µs
#define UART_READBYTE_TIMEOUT 10000    //in µs


#endif /* NODE_CFG_H_ */
