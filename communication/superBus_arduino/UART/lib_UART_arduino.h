/*
 *  lib_UART.h
 *
 *  Created on: 5 mai 2013
 *      Author: quentin
 */

#ifndef LIB_UART_H_
#define LIB_UART_H_

#include "messages.h"

#ifdef __cplusplus
extern "C" { //to enable use in both C projects an C++ projects
#endif


void UART_init(unsigned long speed);
int UART_receive(sMsg *pRet);
int UART_send(sMsg *msg);

#ifdef __cplusplus
}
#endif


#endif /* LIB_UART_H_ */
