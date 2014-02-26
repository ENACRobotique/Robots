/*
 * UDP4bn.h
 *
 *  Created on: 25 févr. 2014
 *      Author: ludo6431
 */

#ifndef UDP4BN_H_
#define UDP4BN_H_

// config files
#include "node_cfg.h"
#include "network_cfg.h"
#include "messages.h"

#if MYADDRD

extern int udpsockfd; // file descriptor

int UDP_init();
int UDP_receive(sMsg *msg);
int UDP_send(const sMsg *msg, bn_Address nextHop);

#endif

#endif /* UDP4BN_H_ */
