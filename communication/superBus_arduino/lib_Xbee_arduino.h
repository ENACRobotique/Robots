/*
 *  lib_xbee.h
 *
 *  Created on: 5 mai 2013
 *      Author: quentin
 */

#ifndef LIB_XBEE_H_
#define LIB_XBEE_H_

#include "messages.h"

void setupXbee();
int Xbee_receive(sMsg *pRet);
int Xbee_send(sMsg msg);

#endif /* LIB_XBEE_H_ */
