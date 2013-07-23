/*
 * blink.cpp
 *
 *  Created on: 4 juin 2013
 *      Author: robot
 */


#include "Arduino.h"
#include "params.h"
#include "lib_superBus.h"
#include "I2C/lib_I2C_arduino.h"

#define SB_PDEBUG

void setup(){

	sb_init(); // Serial.begin(111111) in sb_init()

	// init XBee if using it, won't harm if on an arduino
	pinMode(PIN_DBG_LED,OUTPUT);
    pinMode(5,OUTPUT);
    digitalWrite(5,HIGH);
    delay(100);
    digitalWrite(5,LOW);
    delay(200);
    while(Serial.read() != -1); //flush the buffer for any incorrect bytes

#ifdef SB_PDEBUG
    sb_printDbg(ADDRX_DEBUG, "start turret debug", 0, 0);
#else
	Serial.println("start I2C test");
#endif
}

sMsg msg;
int ledState=0;

void loop(){
	sb_routine();

	if (sb_receive(&msg) > 0){
#ifdef SB_PDEBUG
		sb_printDbg(ADDRX_DEBUG, "got msg", msg.header.type, msg.header.size + sizeof(sGenericHeader));
#else
		Serial.print("got msg w/ type ");
		Serial.println((const char *)eType2str((E_TYPE)msg.header.type));

		if(msg.header.type == E_DEBUG){
			Serial.print("  ");
			Serial.println((char *)msg.payload.debug.msg);
		}
#endif
		ledState^=1;
		digitalWrite(PIN_DBG_LED,ledState);
	}
}
