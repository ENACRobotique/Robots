/*
 * node_cfg.h
 *
 *  Created on: 1 oct. 2013
 *      Author: quentin
 */

#ifndef NODE_CFG_H_
#define NODE_CFG_H_

//network config
#define MYADDRX ADDRX_MAIN
#define MYADDRI ADDRI_MAIN_TURRET
#define MYADDRU 0

#define SB_INC_MSG_BUF_SIZE 4
#define SB_WAIT_XBEE_SND_FAIL    50000

#define ARCH_328P_ARDUINO
#define ARCH_LITTLE_ENDIAN
#define XBEE_WAITFRAME_TIMEOUT 10
#define XBEE_READBYTE_TIMEOUT 10000


#endif /* NODE_CFG_H_ */
