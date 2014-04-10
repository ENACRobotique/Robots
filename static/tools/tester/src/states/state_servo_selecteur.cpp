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
#include "state_servo_selecteur.h"
#include "state_blink.h"

Servo servotest;

sState* testservo_selecteur(){
	static int memPosition=0;
	int Position = (abs(myEnc.read())/2*5)%180;

	if(!digitalRead(SELECT))	//nécessite de valider avant que le servo ne se déplace
		{
			servotest.write(Position);
		}

		if(Position!=memPosition)
		{
			String affich="Angle= "+String(Position);
			Serial.println(affich);
			memPosition=Position;
		}

		if(retour)
		{
			retour=0;
			return(&sMenu_servo);
		}
    return NULL;
}
void initservo_selecteur(sState *prev){
	servotest.attach(9);

}
void deinitservo_selecteur(sState *next){
	servotest.detach();
}

void servo_selecteur(){

}

sState sservo_selecteur={
    0,
    &initservo_selecteur,
    &deinitservo_selecteur,
    &testservo_selecteur
};


