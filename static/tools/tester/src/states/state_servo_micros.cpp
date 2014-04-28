/*
 * state-blink.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include "Arduino.h"
#include "../params.h"
#include "../tools.h"
#include "state_types.h"

#include "state_Menu_principal.h"
#include "state_Menu_servo.h"
#include "state_servo_micros.h"
#include "state_blink.h"

//Servo servotest;

sState* testservo_micros(){
	static int memMicros=0;
	int Micros = (abs(myEnc.read())/2*PRECISION_MICROS)%1800+500;

//	if(!digitalRead(SELECT))	//nécessite de valider avant que le servo ne se déplace
//		{
//			servotest.writeMicroseconds(Micros);
//		}
	servotest.writeMicroseconds(Micros);

		if(Micros!=memMicros)
		{
			String affich="delay= "+String(Micros)+" us";
			Serial.println(affich);
			memMicros=Micros;
		}

		if(retour)
		{
			retour=0;
			return(&sMenu_servo);
		}
    return NULL;
}
void initservo_micros(sState *prev){
	servotest.attach(9);

}
void deinitservo_micros(sState *next){
	servotest.detach();
}

void servo_micros(){

}

sState sservo_micros={
    0,
    &initservo_micros,
    &deinitservo_micros,
    &testservo_micros
};


