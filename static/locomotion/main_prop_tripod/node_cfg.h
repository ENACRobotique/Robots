/*
 * node_cfg.h
 *
 *  Created on: 1 oct. 2013
 *      Author: quentin
 */

#ifndef NODE_CFG_H_
#define NODE_CFG_H_

#include "network_cfg.h"

#ifdef ARCH_LPC21XX
#define MYADDRX 0
#define MYADDRI ADDRI2_MAIN_PROP_POD1
#define MYADDRU ADDRU2_MAIN_PROP
#define MYADDRD 0
#elif defined(ARCH_X86_LINUX)
#define MYADDRX 0
#define MYADDRI 0
#define MYADDRU 0
#define MYADDRD ADDRD1_MAIN_PROP_SIMU
#endif
#define MYADDR (MYADDRX?:MYADDRI?:MYADDRU?:MYADDRD)

#define MYROLE ROLE_PROPULSION
// MYROLE must be equal to role_get_role(MYADDR)

#define BN_INC_MSG_BUF_SIZE 4
#define BN_WAIT_XBEE_SND_FAIL    5000000
#define BN_MAX_RETRIES          2
#define BN_ACK_TIMEOUT          1000    //in ms

// #define ARCH_X86_LINUX       in symbols
// #define ARCH_LITTLE_ENDIAN   in symbols
#define UART_WAITFRAME_TIMEOUT 10       //in µs
#define UART_READBYTE_TIMEOUT 100000    //in µs

#define XBEE_UART_PATH "/dev/ttyUSB0"
#define BN_UART_PATH "/dev/ttyACM0"

#endif /* NODE_CFG_H_ */
