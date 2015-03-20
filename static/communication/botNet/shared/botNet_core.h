/*
 * botNet_core.h
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#ifndef BOTNET_CORE_H_
#define BOTNET_CORE_H_


// config files
#include "bn_inbuf.h"
#include "bn_attach.h"
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
 * MYADDRD : the current node UDP address
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

int bn_init();
int bn_send(sMsg *msg);
int bn_sendLinkcast(sMsg *msg);
int bn_genericSend(sMsg *msg);
int bn_sendAck(sMsg *msg);
int bn_sendRetry(sMsg *msg, int retries);
int bn_routine();
int bn_receive(sMsg *msg);
void bn_route(const sMsg *msg,E_IFACE ifFrom, sRouteInfo *routeInfo);
int bn_forward(const sMsg *msg, E_IFACE ifFrom);


/*
 * bn_isLinkcast : test if an adress is a linkcast one
 * Arguments :
 *      addr : adress to test
 * Return Value :
 *      1 if  the adress is a linkcast one;
 *      0 if it is not.
 */
static inline int bn_isLinkcast(bn_Address addr){
    if ( !(addr & LCAST_SUBNET) || (addr & ~SUBNET_MASK) != (BIT(DEVICE_ADDR_SIZE)-1) ) return 0;
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* BOTNET_CORE_H_ */
