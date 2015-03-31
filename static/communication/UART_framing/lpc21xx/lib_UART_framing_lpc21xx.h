/*
 * lib_UART_framing_lpc21xx.h
 *
 *  Created on: 19 may 2014
 *      Author: Ludo6431
 */

#ifdef ARCH_LPC21XX

#ifndef LIB_UART_FRAMING_LPC21XX_H_
#define LIB_UART_FRAMING_LPC21XX_H_

#include <stdint.h>

int serialInit(uint32_t speed);
int serialDeinit();
int serialRead(uint8_t *byte,uint32_t timeout);
int serialWrite(uint8_t byte);

#endif

#endif
