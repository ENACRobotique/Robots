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
#include "state_servo_deg_validation.h"
#include "lib_IHM.h"


sState* testservo_deg_validation(){
	static int memAngle=servotest.read();
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
	int Angle = max((memAngle+delta*(pos_enc-pos_enc_old)),0);
	Angle=min(Angle,180);
	pos_enc_old=pos_enc;

	if(!digitalRead(SELECT))	//nécessite de valider avant que le servo ne se déplace
	{
		servotest.write(Angle);
	}

	if(Angle!=memAngle)
	{
		afficher("Angle= %d",Angle);
		memAngle=Angle;
	}


	if(!digitalRead(RETOUR))
	{
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_servo);
	}
    return NULL;
}
void initservo_deg_validation(sState *prev){
	servotest.attach(PIN_PWM_SERVO);
	int angle=servotest.read();
	int value_enc=angle*2.0/5.0;
	myEnc.write(value_enc);
	char affich[16];
	snprintf(affich,17,"Angle = %d",angle);
	afficher(affich);
}
void deinitservo_deg_validation(sState *next){
	servotest.detach();
}

void servo_deg_validation(){

}

sState sservo_deg_validation={
    0,
    &initservo_deg_validation,
    &deinitservo_deg_validation,
    &testservo_deg_validation
};


