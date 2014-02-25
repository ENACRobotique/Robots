/*
 * lib_superbus.h
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#ifndef BOTNET_CORE_H_
#define BOTNET_CORE_H_


// config files
#include "network_cfg.h"
#include "messages.h"

// other required libraries

// botNet specific libraries

// standard libraries

#ifdef __cplusplus
extern "C" {
#endif


/*
 * all the following functions requires that the project contains a "node_cfg.h"  file defining the following :
 * MYADDRX : the current node Xbee address
 * MYADDRI : the current node I2C address
 * MYADDRU : the current node UART address
 * MYADDRT : the current node TCP address
 *
 * SB_INC_MSG_BUF_SIZE : size of the buffer for incoming messages
 * ARCH_xxxx : architecture of the node
 */

#ifndef MAX
#define MAX(m, n) ((m)<(n)?(n):(m))
#endif
#ifndef MIN
#define MIN(m, n) ((m)>(n)?(n):(m))
#endif

//function pointeur type for attach function
typedef void(*pfvpm)(sMsg*);

//incoming message & IF structure
typedef struct {
    sMsg msg;       //message
    E_IFACE iFace;  //interface where it has been received
} sMsgIf;

int bn_init();
int bn_send(sMsg *msg);
int bn_genericSend(sMsg *msg);
int bn_sendAck(sMsg *msg);
int bn_routine();
int bn_receive(sMsg *msg);
void bn_route(const sMsg *msg,E_IFACE ifFrom, sRouteInfo *routeInfo);
int bn_forward(const sMsg *msg, E_IFACE ifFrom);
int bn_attach(E_TYPE type,pfvpm ptr);
int bn_deattach(E_TYPE type);
int bn_pushInBufLast(const sMsg *msg, E_IFACE iFace);
sMsgIf * bn_getAllocInBufLast();
int bn_popInBuf(sMsgIf * msg);
sMsgIf *bn_getInBufFirst();
void bn_freeInBufFirst();

#ifdef __cplusplus
}
#endif

#endif /* BOTNET_CORE_H_ */
