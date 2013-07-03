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

#define CBUFF_SIZE 8        //MUST be a power of 2
#define MAX_READ_BYTES 100  //nb of bytes to read before we leave this function (to avoid blocking)

/* handle the reading of the data from the UART on the serial port, checksum test
 * return value : nb of bytes written in pRet, 0 on error (bad checksum) or non-detection of start sequence
 * pointer return : message and type in a structure
 * requires : Serial initialisation
 *
 * Remark : after a call to UART_receive, the memory area designated by pRet may be modified even if no valid message was received
 */
int UART_receive(sMsg *pRet){
    static uint8_t i=0;
    static uint8_t smallBuf[CBUFF_SIZE]={0};
    unsigned int count=0;
    unsigned int j;
    //count to limit the time spend in the loop in case of spam, checksum to get out of the loop if it is correct AND the sender address id OK (if sender=0 it means it has been reset to 0 after reading the message)
    while( Serial.available() \
            && count<=MAX_READ_BYTES \
            &&  ( !cbChecksumHead(smallBuf,CBUFF_SIZE,(i-1)&(CBUFF_SIZE-1)) || !( smallBuf[(i-5)&(CBUFF_SIZE-1)]<<8 | smallBuf[(i-4)&(CBUFF_SIZE-1)] ) ) ) {
        smallBuf[i]=Serial.read();
        i=(i+1)&(CBUFF_SIZE-1);                                          // &7 <~> %8, but better behaviour with negative in our case (and MUCH faster)
        count++;
    }
    if (cbChecksumHead(smallBuf,CBUFF_SIZE,(i-1)&(CBUFF_SIZE-1)) && ( smallBuf[(i-5)&(CBUFF_SIZE-1)]<<8 | smallBuf[(i-4)&(CBUFF_SIZE-1)] ) ){
        count=sizeof(sGenericHeader);

        //we copy the header in the return structure
        for (j=0;j<sizeof(sGenericHeader);j++){
        	((uint8_t *)(&(pRet->header)))[j]=smallBuf[(i-sizeof(sGenericHeader)+j)&(CBUFF_SIZE-1)];
        }

        //clear the "header buffer"
        memset(smallBuf,0,sizeof(smallBuf));

        //we read the rest of the data in this message (given by the "size" field of the header) and write the in the return structure
        //TODO add timeout
        while(count < pRet->header.size+sizeof(sGenericHeader)){
        	if (Serial.available()){
        		pRet->payload.raw[count-sizeof(sGenericHeader)]=Serial.read();
        		count++;
        	}
        }

        //checksum it, if ok then return count, else return 0
        if(checksumPload(pRet)) return count;
        else return 0;
    }

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
