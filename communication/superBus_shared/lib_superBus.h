/*
 * lib_superbus.h
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#ifndef LIB_SUPERBUS_H_
#define LIB_SUPERBUS_H_

#include "messages.h"
#include "network_cfg.h"


/*
 * all the following functions requires that the project contains a "params.h"  file defining the following :
 * MYADDRX : the current node Xbee adress
 * MYADDRI : the current node I2C adress
 *
 * SB_INC_MSG_BUF_SIZE : size of the buffer for incoming messages
 */

int sb_send(sMsg *msg);
int sb_routine();
int sb_receive(sMsg *msg);
E_IFACE sb_route(sMsg *msg,E_IFACE ifFrom);
int sb_forward(sMsg *msg, E_IFACE ifFrom);
int sb_printDbg(sb_Adress dest,char * str,int32_t i, uint32_t u);

#endif /* LIB_SUPERBUS_H_ */
