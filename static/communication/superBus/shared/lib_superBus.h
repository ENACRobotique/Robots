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

#ifdef __cplusplus
extern "C" {
#endif


/*
 * all the following functions requires that the project contains a "params.h"  file defining the following :
 * MYADDRX : the current node Xbee adress
 * MYADDRI : the current node I2C adress
 * MYADDRU : the current node UART adress
 *
 * SB_INC_MSG_BUF_SIZE : size of the buffer for incoming messages
 * ARCH_xxxx : architecture of the node
 */

//function pointeur type for attach function
typedef void(*pfvpm)(sMsg*);

int sb_init();
int sb_send(sMsg *msg);
int sb_routine();
int sb_receive(sMsg *msg);
sRouteInfo sb_route(sMsg *msg,E_IFACE ifFrom);
int sb_forward(sMsg *msg, E_IFACE ifFrom);
int sb_attach(E_TYPE type,pfvpm ptr);


#ifdef __cplusplus
}
#endif

#endif /* LIB_SUPERBUS_H_ */
