/*
 * lib_UART_framing_arduino.cpp
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

#ifdef ARCH_LM4FXX

#include "UART_bufferized.h"
#include "time.h"
#include "../../../global_errors.h"


/* initSerial : Initializes the UART serial interface
 * Argument :
 *  speed : speed in bauds
 * Return value :
 *  0 on success
 *  <0 on error
 */
int serialInit(uint32_t speed){
    uartb_init(speed);
    return 0;
}

/* deinitSerial : Closes the UART serial interface
 * Argument :
 *  none
 * Return value :
 *  0 on success
 *  <0 on error
 */
int serialDeinit(){
    uartb_deinit();
    return 0;
}

/* serialRead : reads one byte from serial interface
 * Argument :
 *  byte : pointer to where the byte should be stored
 *  timeout : in µs, time after which the function returns 0 if no byte can be read.
 * Return value :
 *  1 if one byte have been read
 *  0 if timeout
 *  <0 on error
 *
 */
int serialRead(uint8_t *byte,uint32_t timeout){
    unsigned long int sw=micros();
    int c;

    if (timeout){
        while ( (micros()-sw) <= timeout ) {
            if (uartb_readAvail() ) {
                *byte = uartb_readChar();
                return 1;
            }
        }
    }

    else {
        if (uartb_readAvail()) {
            *byte = uartb_readChar();
            return 1;
        }
    }

    return 0;
}

/* serialWrite : writes a byte on the serial interface
 * Argument :
 *  byte : byte to write
 * Return value :
 *  1 on success
 *  <0 on error
 */
int serialWrite(uint8_t byte){
    if (uartb_writeChar(byte)) return 1;
    return -ERR_UART_WRITE_BYTE;
}

#endif
