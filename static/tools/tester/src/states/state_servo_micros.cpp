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
#include "state_servo_micros.h"
#include "lib_IHM.h"


sState* testservo_micros(){
	static long temps_enc=0;
	static int pos_enc_old=myEnc.read();

	int pos_enc = myEnc.read();

	if(pos_enc!=pos_enc_old){
		if(millis()-temps_enc < DUREE_BIG_STEPS){
			pos_enc = pos_enc_old + 20 * (pos_enc - pos_enc_old);
		}
		else{
			pos_enc = pos_enc_old + 5 * (pos_enc - pos_enc_old);
		}
		pos_enc = CLAMP(500, pos_enc, 3000);
		temps_enc=millis();

		servo_choosen->writeMicroseconds(pos_enc);
		afficher("Micros= %d us", pos_enc);
		myEnc.write(pos_enc);
		pos_enc_old=pos_enc;
	}

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sChoice_servo);
	}
    return NULL;
}

void initservo_micros(sState *prev){
	int micros=servo_choosen->readMicroseconds();
	myEnc.write(micros);
	afficher("Micros= %d us", micros);
}
void deinitservo_micros(sState *next){
}

sState sservo_micros={
    0,
    &initservo_micros,
    &deinitservo_micros,
    &testservo_micros
};
