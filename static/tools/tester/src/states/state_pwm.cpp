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
//int Position = (abs(myEnc.read())/2*5)%260;
	int Position = (myEnc.read()/2*5)%260;

	if(!digitalRead(SELECT))	//nécessite de valider
	{
		while(!digitalRead(SELECT));	//attente du relachement du bouton
		analogWrite(PIN_PWM_SERVO,Position);
	}

	if(Position!=memPosition)
	{
		char affich[16];
		snprintf(affich,17,"Pos= %d /255",Position);
		afficher(affich);
		memPosition=Position;
	}

	/*if(retour)
			{
				retour=0;
				return(&sMenu_principal);
			}*/
	if(!digitalRead(RETOUR))
	{
		delay(3);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_principal);
	}
    return NULL;
}
void initpwm(sState *prev){
	pinMode(PIN_PWM_SERVO,OUTPUT);
	myEnc.write(0);
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


