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
	int Position = (myEnc.read()/2*5)%185;

	servotest.write(Position); //rotation sans validation

	if(Position!=memPosition)
	{

		char affich[16];
		snprintf(affich,17,"Angle= %d",Position);
		afficher(affich);
		memPosition=Position;
	}

	if(!digitalRead(RETOUR))
	{
		delay(3);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_servo);
	}
    return NULL;
}
void initservo_selecter2(sState *prev){
	servotest.attach(PIN_PWM_SERVO);
	int angle=servotest.read();
	int value_enc=5*angle/2;
	myEnc.write(value_enc);

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


