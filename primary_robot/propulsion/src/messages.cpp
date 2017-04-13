/*
 * messages.c
 *
 *  Created on: 10 avr. 2017
 *      Author: guilhem
 */

#include "messages.h"
#include "Arduino.h"
//#include "../propulsion.cpp"

typedef union {

	sMessageDown msg;

	byte data[MSG_DOWN_MAX_SIZE];

} uDownData;

typedef union {
	sMessageUp msg;
	char data[MSG_UP_MAX_SIZE];
} uUpData;



void message_init(int baudrate){
	HWSERIAL.begin(baudrate);
}

uint8_t compute_checksum_down(uDownData msg) {
	uint8_t sum = 0;
	for (int i = DOWN_HEADER_SIZE; i < MSG_DOWN_MAX_SIZE; i++) {
		sum = (sum + msg.data[i]) % 255;
	}
	return sum;
}

int message_recieve(sMessageDown *msg) {
	uDownData raw_data_down;
	uUpData raw_ack_message;
	static uint8_t lastId;
	static boolean isFirstMessage = true;

	if (HWSERIAL.available()) { //If there is some data waiting in the buffer
		int i = 0;

		while (HWSERIAL.available()) { //Read all the data in the buffer (asserting raspi is sending at max one message per teensy loop)

			raw_data_down.data[i] = HWSERIAL.read();

			i++;

		}

		if (compute_checksum_down(raw_data_down) == raw_data_down.msg.checksum) {
			raw_ack_message.msg.type = ACK;
			raw_ack_message.msg.down_id = raw_data_down.msg.id;
			HWSERIAL.write(raw_ack_message.data, MSG_UP_MAX_SIZE);
			if (isFirstMessage || //If it is the first message, accept it
					((raw_data_down.msg.id - lastId)%256>0 && (raw_data_down.msg.id - lastId)%256<128)) { //Check if the message has a id bigger than the last recevied
				lastId = raw_data_down.msg.id;
				*msg = raw_data_down.msg;
				return 1;
			}else{ //This message is an ancient one (previous ACK not received ?)
				return 0;
			}
		} else {
			raw_ack_message.msg.type = NON_ACK;
			raw_ack_message.msg.down_id = raw_data_down.msg.id;
			HWSERIAL.write(raw_ack_message.data, MSG_UP_MAX_SIZE);
			return 0;

		}
	} else {
		return 0; //Serial is empty :'(
	}
}

