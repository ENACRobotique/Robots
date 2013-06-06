/*
 * lib_xbee_x86.h
 *
 *  Created on: 27 mai 2013
 *      Author: quentin
 */

#ifndef LIB_XBEE_X86_H_
#define LIB_XBEE_X86_H_

#include "messages.h"

void Xbee_initSerial(char * devStr);
void Xbee_deInitSerial();
int Xbee_receive(sMsg *pRet);
int Xbee_send(sMsg *msg);

#endif /* LIB_XBEE_X86_H_ */
