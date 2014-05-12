/*
 * bufferized_UART1.h
 *
 *  Created on: 12 mai 2014
 *      Author: quentin
 */

#ifndef BUFFERIZED_UART1_H_
#define BUFFERIZED_UART1_H_

#include "lib_UART_framing_lm4fxx.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"

#define UART_BUFFERIZED_BASE  UART1_BASE
#define UART_BUFFERIZED_FLAGS (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)
#define UART_BUFFERIZED_CBUFF_SIZE  128

void uartb_init(unsigned long speed);
void uartb_deinit();
void uartb_intHandler();
int  uartb_readAvail();
char uartb_readChar();
void uartb_writeChar(char c);
int  uartb_writeCharNonBlocking(char c);

#endif /* BUFFERIZED_UART1_H_ */
