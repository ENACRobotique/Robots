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
#include "state_pwm.h"
#include "state_blink.h"

// test;

sState* testpwm(){
	static int memPosition=0;
	int Position = (abs(myEnc.read())/2*5)%260;

	if(!digitalRead(SELECT))	//nécessite de valider
		{
			analogWrite(PIN_PWM_SERVO,Position);

		}

		if(Position!=memPosition)
		{
			String affich="pos= "+String(Position)+"/255";
			Serial.println(affich);
			memPosition=Position;
		}

		if(retour)
		{
			retour=0;
			return(&sMenu_principal);
		}
    return NULL;
}
void initpwm(sState *prev){
	pinMode(PIN_PWM_SERVO,OUTPUT);
}
void deinitpwm(sState *next){

}

void pwm(){

}

sState spwm={
    0,
    &initpwm,
    &deinitpwm,
    &testpwm
};


