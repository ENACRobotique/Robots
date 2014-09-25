/*
 * lib_UART_framing_lpc21xx.c
 *
 *  Created on: 19 may 2014
 *      Author: Ludo6431
 */

#ifdef ARCH_LPC21XX

#include <uart0.h>
#include <sys_time.h>
#include "../../../global_errors.h"

#include "lib_UART_framing_lpc21xx.h"

/* serialInit : Initializes the UART serial interface
 * Argument :
 *  speed : speed in bauds
 * Return value :
 *  0 on success
 *  <0 on error
 */
int serialInit(uint32_t speed){
    uart0_init(speed);
    return 0;
}

/* serialDeinit : Closes the UART serial interface
 * Argument :
 *  none
 * Return value :
 *  0 on success
 *  <0 on error
 */
int serialDeinit(){
    //TODO
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
int serialRead(uint8_t *byte, uint32_t timeout){
    unsigned long int sw=micros();

    if(timeout){
        while((micros()-sw) <= timeout){
            if(UARTReadAvailable()){
                *byte = UARTReadChar();
                return 1;
            }
        }
    }
    else{
        if(UARTReadAvailable()){
            *byte = UARTReadChar();
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
    UARTWriteChar(byte);
    return 1;
}

#endif
