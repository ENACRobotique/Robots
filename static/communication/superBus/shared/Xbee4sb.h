/*
 * Xbee4sb.h
 *
 *  Created on: 28 juin 2013
 *      Author: quentin
 */

#ifndef XBEE4SB_H_
#define XBEE4SB_H_

#ifdef __cplusplus
extern "C" { //to enable use in both C projects an C++ projects
#endif

#include "messages.h"
#include "Xbee_API.h"

int Xbee_setup();
int Xbee_receive(sMsg *pRet);
int Xbee_send(sMsg *msg, sb_Address nexthop);

#ifdef __cplusplus
}
#endif

#endif /* XBEE4SB_H_ */
