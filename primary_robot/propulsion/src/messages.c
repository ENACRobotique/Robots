/*
 * messages.c
 *
 *  Created on: 10 avr. 2017
 *      Author: guilhem
 */

#include "messages.h"

typedef union{

  sMessage msg;

  byte data[MSG_MAX_SIZE];

}uData;

uData raw_data;


int message_recieve(sMessage *msg){

  if (SERIAL.available()){

    digitalWrite(led, HIGH);

    int i = 0;

    while (SERIAL.available()){

      raw_data.data[i] = Serial1.read();

      i++;

    }


    //TODO : CHECK CHECK SUM

    //TODO : SEND ACK OR NOT

    *msg = raw_data.msg;

    return 1;

  }

}

