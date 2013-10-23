/*
 * lib_UART_x86.h
 *
 *  Created on: 27 mai 2013
 *      Author: quentin
 */

#ifndef LIB_UART_X86_H_
#define LIB_UART_X86_H_

#include "messages.h"

extern int UART_serial_port;

void setupUART();
void UART_initSerial(char * devStr);
void UART_deInitSerial();
int UART_receive(sMsg *pRet);
int UART_send(sMsg *msg);

#endif /* LIB_UART_X86_H_ */
