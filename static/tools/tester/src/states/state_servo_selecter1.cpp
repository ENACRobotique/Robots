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
#include "state_servo_selecter1.h"
#include "state_blink.h"

//Servo servotest;

sState* testservo_selecter1(){
	static int memPosition=0;
	int Position = (abs(myEnc.read()-deltaenc)/2*5)%185;

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
			memenc=myEnc.read();
			return(&sMenu_servo);
		}
    return NULL;
}
void initservo_selecter1(sState *prev){
	deltaenc=myEnc.read()-memenc;
	servotest.attach(PIN_PWM_SERVO);

}
void deinitservo_selecter1(sState *next){
	servotest.detach();
}

void servo_selecter1(){

}

sState sservo_selecter1={
    0,
    &initservo_selecter1,
    &deinitservo_selecter1,
    &testservo_selecter1
};


