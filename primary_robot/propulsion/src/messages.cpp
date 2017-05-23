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
	byte data[MSG_UP_MAX_SIZE];
} uUpData;

static boolean isFirstMessage;
static uint8_t lastId;


void message_init(int baudrate){
	HWSERIAL.begin(baudrate);
	isFirstMessage = true;
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

	if (HWSERIAL.available()) { //If there is some data waiting in the buffer

		if (HWSERIAL.available() >= MSG_DOWN_MAX_SIZE) { //Read all the data in the buffer (asserting raspi is sending at max one message per teensy loop)
			for (int i = 0; i < MSG_DOWN_MAX_SIZE; i++){
				raw_data_down.data[i] = HWSERIAL.read();
			}
		}else{
			return 0;
		}

		if (compute_checksum_down(raw_data_down) == raw_data_down.msg.checksum) {
			raw_ack_message.msg.type = ACK;
			raw_ack_message.msg.down_id = raw_data_down.msg.id;
			HWSERIAL.write(raw_ack_message.data, MSG_UP_MAX_SIZE);


			if (isFirstMessage || //If it is the first message, accept it
					raw_data_down.msg.type == RESET ||
					((raw_data_down.msg.id - lastId)%256>0 && (raw_data_down.msg.id - lastId)%256<128)) { //Check if the message has a id bigger than the last recevied
				isFirstMessage = false;
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

int message_send(sMessageUp msg){
	uUpData raw_data_up;
	raw_data_up.msg = msg;
	HWSERIAL.write(raw_data_up.data, MSG_UP_MAX_SIZE);
	return 0;
}

