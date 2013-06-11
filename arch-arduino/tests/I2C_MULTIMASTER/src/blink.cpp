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

void setup(){

	sb_init();

	//debug over serial;
	Serial.begin(115200);

	pinMode(PIN_DBG_LED,OUTPUT);

}

void loop(){
	//static
	static unsigned long prevmillis=millis();
	static int ledState=0;
	static int nBytes=0;
	//variables
	sMsg msg;


//	nBytes+=sb_routine();

//	nBytes += ;

	if ((millis()-prevmillis) > 2000){
		prevmillis=millis();

		Serial.print("nBytes : ");
		Serial.println(I2C_receive(&msg));
		nBytes=0;
		ledState^=1;
		digitalWrite(PIN_DBG_LED,ledState);
	}
}
