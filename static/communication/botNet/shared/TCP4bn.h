/*
 * TCP4bn.h
 *
 *  Created on: 25 févr. 2014
 *      Author: ludo6431
 */

#ifndef TCP4BN_H_
#define TCP4BN_H_

// config files
#include "node_cfg.h"
#include "network_cfg.h"
#include "messages.h"

#if MYADDRT

int TCP_init();
int TCP_receive(sMsg *msg);
int TCP_send(const sMsg *msg);

#endif

#endif /* TCP4BN_H_ */
