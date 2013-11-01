/*
 * lib_UART_framing_arduino.h
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

#ifndef LIB_UART_FRAMING_ARDUINO_H_
#define LIB_UART_FRAMING_ARDUINO_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int serialInit(uint32_t speed);
int serialDeinit();
int serialRead(uint8_t *byte,uint32_t timeout);
int serialWrite(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* LIB_UART_FRAMING_ARDUINO_H_ */
