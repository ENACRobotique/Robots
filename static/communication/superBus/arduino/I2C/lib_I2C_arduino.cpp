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
#include "node_cfg.h"
#include "lib_superBus.h"
#include "Arduino.h"

/*
 * puts the received message directly in the incoming message buffer
 */
void receiveEvent(int i){
    sMsgIf *tmp;
    int j;


    tmp=sb_getAllocInBufLast();

    //if there is no space, trash the incoming message
    if ( tmp==NULL ){
        for (j=0;j<i;j++){
            Wire.read();
        }
        return;
    }
    //else, put message in incoming message buffer
    else {
        Wire.readBytes((char*)&(tmp->msg),i);
        tmp->iFace=IF_I2C;
    }
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
 * dummy function, not used because message are stored in the event (cf receiveEvent)
 */
int I2C_receive(sMsg *pRet){
    return 0;
}

/*
 * Handle the sending of the message to the I²C interface
 * Size & checksum of msg must be set before calling txXbee
 * Argument :
 *  msg : message to send (thank captain obvious!)
 *  firstDest : destination ON THE SENDER'S I2C BUS (in case of routing, may differ from msg->header.destAddr )
 * Return value :
 *  number of bytes writen (-1 if error)
 */
int I2C_send(const sMsg *msg, sb_Address firstDest){

    //these two variables are here to ensure that enough time was spent between the current sending of data and the previous one
    static unsigned long prevSend=0,delay=0;
    int count=0;

    //we wait to let enough time to an arduino receiver to receive the message
    while( (micros()-prevSend)<delay );

    Wire.beginTransmission( (int)(firstDest & DEVICEI_MASK)>>1 );
    count=Wire.write((const uint8_t *)msg,msg->header.size+sizeof(sGenericHeader));

    prevSend=micros();
    delay=((msg->header.size+sizeof(sGenericHeader)+2)*37);//in µs, based on experimental measurement, 2.4 ms required for 66 Bytes (header+Pload+I2C address)


    if( Wire.endTransmission() ){
    	return -1;
    }
    return count;
}
