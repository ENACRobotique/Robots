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
#include "state_servo_selecter2.h"
#include "state_blink.h"

//Servo servotest;

sState* testservo_selecter2(){
	static int memPosition=0;
	int Position = (abs(myEnc.read()-deltaenc)/2*5)%185;

	servotest.write(Position); //rotation sans validation

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
void initservo_selecter2(sState *prev){
	deltaenc=myEnc.read()-memenc;
	servotest.attach(PIN_PWM_SERVO);

}
void deinitservo_selecter2(sState *next){
	servotest.detach();
}

void servo_selecter2(){

}

sState sservo_selecter2={
    0,
    &initservo_selecter2,
    &deinitservo_selecter2,
    &testservo_selecter2
};


