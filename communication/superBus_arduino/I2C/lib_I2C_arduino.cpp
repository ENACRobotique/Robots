/*
 * lib_I2C_arduino.cpp
 *
 *  Created on: 6 juin 2013
 *      Author: quentin
 */

#include "Wire.h"
#include "network_cfg.h"
#include "lib_checksum.h"
#include "lib_I2C_arduino.h"


#define CBUFF_SIZE 8        //MUST be a power of 2
#define MAX_READ_BYTES 100  //nb of bytes to read before we leave this function (to avoid blocking)

/* handle the reading of the data from the I2C on the serial port, checksum test
 * return value : nb of bytes written in pRet, 0 on error (bad checksum) or non-detection of start sequence
 * pointer return : message and type in a structure
 * requires : Wire.begin(address) initialisation
 *
 * Remark : after a call to Xbee_receive, the memory area designated by pRet may be modified even if no valid message was received
 */
int I2C_receive(sMsg *pRet){
    static int i=0;

    //reading the header
    while (i<sizeof(sGenericHeader) && Wire.available()){
        ((uint8_t *)pRet)[i]=Wire.read();
    }

    //reading the rest of the message, according to the size written in header
    while (i<sizeof(sGenericHeader)+pRet->header.size && Wire.available()){
            ((uint8_t *)pRet)[i]=Wire.read();
    }

    if (checksumHead( &(pRet->header )) && checksumPload(pRet)) return i;
    return 0;
}


/*
 * Handle the sending of the message to the xbee via the serial.
 * Size & checksum of msg must be set before calling txXbee
 * Argument :
 *  msg : message to send (thank captain obvious!)
 * Return value :
 *  number of bytes writen (0 if error)
 */
int I2C_send(sMsg *msg){
    int count=0;

    Wire.beginTransmission( (int)(msg->header.destAddr & DEVICEI_MASK)>>1 );
    count=Wire.write((uint8_t *)msg,msg->header.size+sizeof(sGenericHeader));
    Wire.endTransmission();

    return count;
}
