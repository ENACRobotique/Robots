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

  sMessageDown msg;

  byte data[MSG_DOWN_MAX_SIZE];

}uDownData;

typedef union{
	sMessageUp msg;
	byte data[MSG_UP_MAX_SIZE];
}uUpData;

uDownData raw_data_down;
uUpData raw_data_up;


int message_recieve(sMessageDown *msg){

  if (HWSERIAL.available()){

    digitalWrite(13, HIGH);

    int i = 0;

    while (HWSERIAL.available()){

      raw_data_down.data[i] = HWSERIAL.read();

      i++;

    }


    //TODO : CHECK CHECK SUM

    //TODO : SEND ACK OR NOT

    *msg = raw_data_down.msg;

    return 1;

  }

}

