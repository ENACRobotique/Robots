/*
 *  lib_xbee.h
 *
 *  Created on: 5 mai 2013
 *      Author: quentin
 */

#ifndef LIB_XBEE_H_
#define LIB_XBEE_H_

#include "messages.h"

#ifdef __cplusplus
extern "C" { //to enable use in both C projects an C++ projects
#endif

void setupXbee();
int Xbee_receive(sMsg *pRet);
int Xbee_send(sMsg msg);

#ifdef __cplusplus
}
#endif


#endif /* LIB_XBEE_H_ */
