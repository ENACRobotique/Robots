/*
 * Xbee4sb.h
 *
 *  Created on: 28 juin 2013
 *      Author: quentin
 */

#ifndef XBEE4BN_H_
#define XBEE4BN_H_

#ifdef __cplusplus
extern "C" { //to enable use in both C projects an C++ projects
#endif

#include "messages.h"
#include "../../Xbee_API/shared/Xbee_API.h"

int Xbee_setup();
int Xbee_receive(sMsg *pRet);
int Xbee_send(const sMsg *msg, bn_Address nexthop);

#ifdef __cplusplus
}
#endif

#endif /* XBEE4BN_H_ */
