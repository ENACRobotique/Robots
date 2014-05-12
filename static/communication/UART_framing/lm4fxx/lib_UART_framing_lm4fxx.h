/*
 * lib_UART_framing_lm4fxx.h
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

#ifdef ARCH_LM4FXX

#ifndef LIB_UART_FRAMING_LM4FXX_H_
#define LIB_UART_FRAMING_LM4FXX_H_



#include <stdint.h>

int serialInit(uint32_t speed);
int serialDeinit();
int serialRead(uint8_t *byte,uint32_t timeout);
int serialWrite(uint8_t byte);


#endif

#endif
