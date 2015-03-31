#ifndef _UART0_H
#define _UART0_H

#include <stdint.h>

#define UART_CBUFF_SIZE (128)

void        UARTWriteChar       (uint8_t ch);
uint8_t     UARTReadChar        ();
int         UARTReadAvailable   ();

unsigned int uart0_init(unsigned int baud);

#endif
