/*
 * bn_inbuf.h
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#ifndef LIB_BOTNET_SHARED_BN_INBUF_H_
#define LIB_BOTNET_SHARED_BN_INBUF_H_

#include <messages.h>
#include <network_cfg.h>

#ifdef __cplusplus
extern "C" {
#endif

//incoming message & IF structure
typedef struct {
    sMsg msg;       //message
    E_IFACE iFace;  //interface where it has been received
} sMsgIf;

int         bn_pushInBufLast        (const sMsg *msg, E_IFACE iFace);
sMsgIf *    bn_getAllocInBufLast    ();
int         bn_popInBuf             (sMsgIf * msg);
sMsgIf *    bn_getInBufFirst        ();
void        bn_freeInBufFirst       ();

#ifdef __cplusplus
}
#endif

#endif /* LIB_BOTNET_SHARED_BN_INBUF_H_ */
