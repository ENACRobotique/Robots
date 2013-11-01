/*
 * lib_UART.cpp
 *
 *  Created on: 5 mai 2013
 *      Author: quentin
 */

#include "messages.h"
#include "lib_checksum.h"
#include "Arduino.h"
#include "lib_UART_arduino.h"

//#define DEBUG_UART



void UART_init(unsigned long speed){
	Serial.begin(speed);
}


/* handle the reading of the data from the UART on the serial port, checksum test
 * return value : nb of bytes written in pRet, 0 on error (bad checksum) or non-detection of start sequence
 * pointer return : message and type in a structure
 * requires : Serial initialisation
 *
 * Remark : after a call to UART_receive, the memory area designated by pRet may be modified even if no valid message was received
 */
int UART_receive(sMsg *pRet){
    //TODO

    return 0;
}


/*
 * Handle the sending of the message to the UART via the serial.
 * Size & checksum of msg must be set before calling txUART
 * Argument :
 *  msg : message to send (thank captain obvious!)
 * Return value :
 *  number of bytes writen (0 if error)
 */
int UART_send(sMsg *msg){
    return  Serial.write((uint8_t *)msg,msg->header.size+sizeof(sGenericHeader));
}
