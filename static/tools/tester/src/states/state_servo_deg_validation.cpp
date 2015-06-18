/*
 * state-blink.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include "Arduino.h"
#include "params.h"
#include "tools.h"
#include "state_types.h"

#include "state_choice_servo.h"
#include "state_servo_deg_validation.h"
#include "lib_IHM.h"


sState* testservo_deg_validation(){
	static long temps_enc=0;
	static int pos_enc_old=myEnc.read();

	int pos_enc = myEnc.read();

	if(pos_enc!=pos_enc_old){
		if(millis()-temps_enc < DUREE_BIG_STEPS)
		{
			pos_enc = pos_enc_old + 10 * (pos_enc - pos_enc_old);
		}
		pos_enc = CLAMP(0, pos_enc, 180);
		temps_enc=millis();

		afficher("Angle= %d", pos_enc);
		myEnc.write(pos_enc);
		pos_enc_old=pos_enc;
	}

	if(!digitalRead(SELECT)){	//nécessite de valider avant que le servo ne se déplace
		while(!digitalRead(SELECT));
		servo_choosen->write(pos_enc);
	}

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sChoice_servo);
	}
    return NULL;
}
void initservo_deg_validation(sState *prev){
	int angle=servo_choosen->read();		//work only if the servo was already attach and used.
	myEnc.write(angle);
	afficher("Angle= %d", angle);
}

void deinitservo_deg_validation(sState *next){
}

sState sservo_deg_validation={
    0,
    &initservo_deg_validation,
    &deinitservo_deg_validation,
    &testservo_deg_validation
};
