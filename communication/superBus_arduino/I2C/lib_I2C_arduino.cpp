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
#include "params.h"
#include "lib_superBus.h"
#include "Arduino.h"

// must be a power of 2
#define NB_I2C_OUT 4

sMsg i2cincBuf[NB_I2C_OUT]={{{0}}};
int i2ciFirst=0,i2ciNext=0,i2cnbMsg=0;

void receiveEvent(int i){
	if (i2ciFirst==i2ciNext && i2cnbMsg==NB_I2C_OUT) {
		i2ciFirst=(i2ciFirst+1)&(NB_I2C_OUT-1); //"drop" oldest message if buffer is full
		i2cnbMsg--;
	}
	Wire.readBytes((char*)&(i2cincBuf[i2ciNext]),i);
	i2ciNext=(i2ciNext+1)&(NB_I2C_OUT-1);
	i2cnbMsg++;
}

/*
 * initializes the I²C interface on ATmega
 */
void I2C_init(unsigned long speed){
    Wire.begin((MYADDRI & 0xff) >> 1);
    TWBR = ((F_CPU / speed) - 16)>>1; // set speed

    Wire.onReceive(receiveEvent);
}

/*
 * handle the reading of the data from the hardware I²C interface, checksum test
 * return value : nb of bytes written in pRet, 0 on error (bad checksum) or non-detection of start sequence
 * pointer return : message and type in a structure
 *
 * Remark : after a call to this function, the memory area designated by pRet may be modified even if no valid message was received
 */
int I2C_receive(sMsg *pRet){
	if ( i2ciFirst==i2ciNext && !i2cnbMsg) return 0;

	//pop the oldest message of incoming buffer and updates index
	memcpy(pRet, &(i2cincBuf[i2ciFirst]), i2cincBuf[i2ciFirst].header.size + sizeof(sGenericHeader));
	i2ciFirst=(i2ciFirst+1)&(NB_I2C_OUT-1);
	i2cnbMsg--;

	return (pRet->header.size + sizeof(sGenericHeader));
}

/*
 * Handle the sending of the message to the I²C interface
 * Size & checksum of msg must be set before calling txXbee
 * Argument :
 *  msg : message to send (thank captain obvious!)
 *  firstDest : destination ON THE SENDER'S I2C BUS (in case of routing, may differ from msg->header.destAddr )
 * Return value :
 *  number of bytes writen (0 if error)
 */
int I2C_send(const sMsg *msg, sb_Address firstDest){
    int count=0;

    Wire.beginTransmission( (int)(firstDest & DEVICEI_MASK)>>1 );
    count=Wire.write((const uint8_t *)msg,msg->header.size+sizeof(sGenericHeader));
    Wire.endTransmission();

    return count;
}
