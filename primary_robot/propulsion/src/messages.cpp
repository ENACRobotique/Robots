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


uint8_t compute_checksum_down(uDownData msg){
	uint8_t sum = 0;
	for (int i = DOWN_HEADER_SIZE; i < MSG_DOWN_MAX_SIZE; i++){
		sum = (sum + msg.data[i]) % 255;
	}
	return sum;
}

int message_recieve(sMessageDown *msg){

  if (HWSERIAL.available()){

    digitalWrite(13, HIGH);

    int i = 0;

    while (HWSERIAL.available()){

      raw_data_down.data[i] = HWSERIAL.read();

      i++;

    }


    if (compute_checksum_down(raw_data_down) == raw_data_down.msg.checksum){
    	// TODO : send ACK
    	//TODO : check message id
        *msg = raw_data_down.msg;
        return 1;
    }else{
    	// TODO : send NOT ACK
    	return 0;

    }
  }else{
	  return -1; //Serial not available
  }
}



