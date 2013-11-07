/*
 * lib_UART_framing.h
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

#ifndef LIB_UART_FRAMING_H_
#define LIB_UART_FRAMING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int UART_init(const char* device, uint32_t speed);
int UART_deinit(const char* device);
int UART_writeFrame(const void *pt,int size);
int UART_readFrame(void *pt,int maxsize);
int serialWriteEscaped(uint8_t byte);
int serialReadEscaped(uint8_t *byte, uint32_t timeout);


//// Special char
#define UART_FRAME_START    0x7E
#define UART_ESCAPE_CHAR    0x7D
#define UART_ESCAPE_MASK    0x20
#define UART_XON            0x11
#define UART_XOFF           0x13

#define UART_MTU            128

#ifdef __cplusplus
}
#endif

#endif /* LIB_UART_FRAMING_H_ */
