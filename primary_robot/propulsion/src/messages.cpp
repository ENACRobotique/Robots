/*
 * messages.c
 *
 *  Created on: 10 avr. 2017
 *      Author: guilhem
 */

#include "messages.h"
#include "Arduino.h"
//#include "../propulsion.cpp"

typedef union{

  sMessage msg;

  byte data[MSG_MAX_SIZE];

}uData;

uData raw_data;


int message_recieve(sMessage *msg){

  if (HWSERIAL.available()){

    digitalWrite(13, HIGH);

    int i = 0;

    while (HWSERIAL.available()){

      raw_data.data[i] = HWSERIAL.read();

      i++;

    }


    //TODO : CHECK CHECK SUM

    //TODO : SEND ACK OR NOT

    *msg = raw_data.msg;

    return 1;

  }

}

