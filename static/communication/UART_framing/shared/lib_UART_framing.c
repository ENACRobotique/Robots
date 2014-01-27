/*
 * lib_UART_framing.c
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

/*
 * UART_framing library.
 * Frame-oriented level 2 communication protocol over UART.
 * Matches the use of Xbee API mode 2 UART communication.
 *
 * Frame structure :
 * | start character | size of data (MSB) | size of data (LSB) | data (first byte) |...| data (last byte) | checksum |
 *
 * Checksum :
 *  To calculate: Not including frame delimiters and length, add all bytes keeping only the lowest 8 bits of the result and subtract from 0xFF.
 *  To verify: Add all bytes (include checksum, but not the delimiter and length). If the checksum is correct, the sum will equal 0xFF.
 */


// config files
#include "node_cfg.h"

// other required libraries
#include "../../../global_errors.h"

// UART_FRAMING specific libraries
#include "lib_UART_framing.h"
#ifdef ARCH_328P_ARDUINO
#include "../arduino/lib_UART_framing_arduino.h"
#elif defined(ARCH_X86_LINUX)
#include "../linux/lib_UART_framing_linux.h"
#else
#error "in UART_framing lib, no known arch symbol defined"
#endif

// standard libraries


int UART_init(const char* device, uint32_t option){
    int retval=0;
#ifdef ARCH_328P_ARDUINO
    retval=serialInit(option);
#elif defined(ARCH_X86_LINUX)
    retval=serialInit(device,option);
#endif
    if (retval>=0) return 0;
    else return -ERR_UART_INIT_FAILED;
}

int UART_deinit(const char* device){
    if (serialDeinit()>=0) return 0;
    else return -ERR_UART_DEINIT_FAILED;
}

/* UART_readFrame writes an UART frame on the wire
 * Argument :
 *  pt : pointeur to the memory area where the frame will be read.
 *  size : size of data to read at pt.
 * Return value :
 *  >0 : nb of bytes written on success (excluding escape char and UART_framing overhead)
 *  <0 if error (ex : writing error, size too big)
 */
int UART_writeFrame(const void *pt,int size){
    uint8_t cSum=0; //checksum
    int j=0;
    int ret;

    //check size :
    if ( size >= (UART_MTU) ) return -ERR_UART_OVERSIZE;

    //write start byte
    if ( (ret=serialWrite(UART_FRAME_START)) <= 0 ) return ret;

    //write size
    if ( (ret=serialWriteEscaped((uint8_t)(size>>8))) <= 0 ) return ret;   //MSB
    if ( (ret=serialWriteEscaped((uint8_t)(size&0xff))) <= 0 ) return ret; //LSB

    //write data
    for (j=0;j<size;j++){
        if ( (ret=serialWriteEscaped(((uint8_t*)pt)[j])) <= 0 ) return ret;
        cSum+=((uint8_t*)pt)[j];
    }

    //compute and write checksum
    cSum=0xff-cSum;
    if ( (ret=serialWriteEscaped(cSum)) <= 0 ) return ret;

    return size;
}


/* UART_readFrame : reads an UART frame if available
 * Argument :
 *  pt : pointeur to the memory area where the frame will be written.
 *  maxsize : maximum number of bytes written at pt.
 * Return value :
 *  >0 : nb of bytes written (success)
 *  0 : there was no frame to read (normal behavior when nothing to read)
 *  <0 if error. Example :
 *      checksum error
 *      size read too big, WARNING : in this case, the frame is dropped.
 *      reading error (timeout of expected byte or else)
 *
 *  WARNING : data will be written at pt even if checksum is wrong. Testing return value is strongly recommended.
 */
int UART_readFrame(void *pt, int maxsize){
    uint8_t byte=0,byte1=0;
    int ret=0;
    int size=0;
    uint8_t cSum=0; //checksum
    int j;

    //waiting for a start character. If something else, read again
    do {
        if ( (ret=serialRead(&byte,UART_WAITFRAME_TIMEOUT)) <=0 ) return ret; //nothing to read here is not an error (normal behavior for non blocking functions if there is not message to read)
    }while (byte!=UART_FRAME_START);

frame_start :
    //read size of frame
    //(for EVERY byte, test if error AND test if start byte)
    if ( (ret=serialReadEscaped(&byte,UART_READBYTE_TIMEOUT)) <= 0 ) return (ret==0)?-ERR_UART_READ_BYTE_TIMEOUT:ret;
    if ( byte == UART_FRAME_START && ret == 1) goto frame_start;
    if ( (ret=serialReadEscaped(&byte1,UART_READBYTE_TIMEOUT)) <= 0 ) return (ret==0)?-ERR_UART_READ_BYTE_TIMEOUT:ret;
    if ( byte == UART_FRAME_START && ret == 1) goto frame_start;

    size=(byte<<8)+byte1;

    //test maxSize if too big for destination buffer, read the message bytes and checksum to clear the UART link of them
    if ( size > maxsize ){
        for (j=0;j<size+1;j++){
            if ( (ret=serialReadEscaped(&byte,UART_READBYTE_TIMEOUT)) <= 0 ) return (ret==0)?-ERR_UART_READ_BYTE_TIMEOUT:ret;
            if ( byte == UART_FRAME_START && ret == 1) goto frame_start;
        }
        return -ERR_UART_OVERSIZE;
    }

    //read the data bytes (and start computing for the test of checksum)
    for (j=0;j<size;j++){
        if ( (ret=serialReadEscaped(&byte,UART_READBYTE_TIMEOUT)) <= 0 ) return (ret==0)?-ERR_UART_READ_BYTE_TIMEOUT:ret;;
        if ( byte == UART_FRAME_START && ret == 1) goto frame_start;
        ((uint8_t*)pt)[j]=byte;
        cSum+=byte;
    }

    //read the checksum byte
    if ( (ret=serialReadEscaped(&byte,UART_READBYTE_TIMEOUT)) <= 0 ) return (ret==0)?-ERR_UART_READ_BYTE_TIMEOUT:ret;
    if ( byte == UART_FRAME_START && ret == 1) goto frame_start;
    cSum+=byte;
    if (cSum==0xff) return size;
    else return -ERR_UART_CSUM;
}

/* XbeeWriteEscaped : writes `byte` on the serial link, including escaping if needed
 * Argument :
 *  byte : byte to write
 * Return value :
 *  1 if write succeed,
 *  <0 otherwise
 */
int serialWriteEscaped(uint8_t byte){
    if ( byte == UART_FRAME_START || byte == UART_XOFF || byte == UART_XON || byte == UART_ESCAPE_CHAR ){
        int ret;
        if ((ret=serialWrite(UART_ESCAPE_CHAR))==1 && (ret=serialWrite( byte^UART_ESCAPE_MASK ))==1) return 1;
        return ret;
    }
    else return serialWrite(byte);
}

/* XbeeReadEscaped : writes one byte on the serial link, removing escaping if needed
 * Return value :
 *  1 if read succeed,
 *  2 if an escaping occured
 *  0 if there was nothing to read
 *  <0 on error while reading
 */
int serialReadEscaped(uint8_t *byte, uint32_t timeout){
    uint8_t byteRed=0;
    int ret;

    if ( (ret=serialRead(&byteRed,timeout)) <= 0 ) return ret;

    if ( byteRed == UART_ESCAPE_CHAR ){
        if ( (ret=serialRead(&byteRed,timeout)) <= 0 ) return (ret==0)?-ERR_UART_READ_BYTE_TIMEOUT:ret;
        *byte=byteRed^UART_ESCAPE_MASK;
        return 2;
    }

    else *byte=byteRed;
    return 1;

}



