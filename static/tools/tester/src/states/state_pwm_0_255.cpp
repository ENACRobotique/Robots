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
#include "state_Menu_pwm.h"
#include "state_pwm_0_255.h"
#include "state_blink.h"


sState* testpwm_0_255(){
/*	static int memPosition=0;
	int Position = (myEnc.read()/2*5)%260;
	if (Position<0)
	{
		Position=0;
		myEnc.write(0);
	}

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
*/
	static int memValue=0;
	static long temps_enc=0;
	static int pos_enc_old=myEnc.read();

	int pos_enc=myEnc.read()/2;
	if (pos_enc < 0)
	{
		pos_enc=0;
		myEnc.write(0);
	}
	int delta=1;

	if(pos_enc!=pos_enc_old)
	{
		long deltat=millis()-temps_enc;
		if(deltat<DUREE_BIG_STEPS)
		{
			delta=10;
		}
		temps_enc=millis();
	}
	int Value = max((memValue+delta*(pos_enc-pos_enc_old)),0);
	Value=min(Value,255);
	pos_enc_old=pos_enc;

	if(!digitalRead(SELECT))	//nécessite de valider avant que le servo ne se déplace
	{
		analogWrite(PIN_PWM_SERVO,Value);
	}

	if(Value!=memValue)
	{
		char affich[16];
		snprintf(affich,17,"Pos= %d /255",Value);
		afficher(affich);
		memValue=Value;
	}

	if(!digitalRead(RETOUR))
	{
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_pwm);
	}
    return NULL;
}
void initpwm_0_255(sState *prev){
	pinMode(PIN_PWM_SERVO,OUTPUT);
	myEnc.write(0);
	analogWrite(PIN_PWM_SERVO,0);
	char affich[16];
	snprintf(affich,17,"Pos= 0 /255");
	afficher(affich);
}
void deinitpwm_0_255(sState *next){

}

void pwm_0_255(){

}

sState spwm_0_255={
    0,
    &initpwm_0_255,
    &deinitpwm_0_255,
    &testpwm_0_255
};


