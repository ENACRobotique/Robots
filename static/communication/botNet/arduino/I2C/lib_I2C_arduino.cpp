/*
 * lib_I2C_arduino.cpp
 *
 *  Created on: 6 juin 2013
 *      Author: quentin
 */


#ifdef ARCH_328P_ARDUINO

// config files
#include "network_cfg.h"
#include "node_cfg.h"

// other required libraries
#include "../../shared/bn_checksum.h"
#include "global_errors.h"

// I2C_arduino specific libraries
#include "lib_I2C_arduino.h"
#include "../../../../core/arduino/libraries/Wire/Wire.h"
#include "../../shared/botNet_core.h"

// standard libraries
#include "Arduino.h"

/*
 * puts the received message directly in the incoming message buffer
 */
void receiveEvent(int i){
    sMsgIf *tmp;

    tmp=bn_getAllocInBufLast();
    if(tmp) {
        Wire.readBytes((char*)&tmp->msg,MIN((unsigned int)i,sizeof(tmp->msg)));
        tmp->iFace=IF_I2C;
    }

    while(Wire.read()>=0);
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
int I2C_send(const sMsg *msg, bn_Address firstDest){
    int ret;

    Wire.beginTransmission( (int)(firstDest & DEVICEI_MASK)>>1 );
    ret=Wire.write((const uint8_t *)msg,msg->header.size+sizeof(sGenericHeader));
    if( Wire.endTransmission()!=0 ){
        return -ERR_I2C_END_TX;
    }
    return ret;
}

#endif
