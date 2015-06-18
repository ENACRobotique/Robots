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
	static int memMicros=500;
	static long temps_enc=0;
	static int pos_enc_old=0;

	int pos_enc=abs(myEnc.read())/2;

	if(pos_enc!=pos_enc_old)
	{
		long deltat=millis()-temps_enc;
		if(deltat<DUREE_BIG_STEPS)
		{
			pos_enc = pos_enc_old + 10 * (pos_enc - pos_enc_old);
			myEnc.write(pos_enc);
		}
		temps_enc=millis();
	}
	int Micros = CLAMP(500, memMicros+PRECISION_MICROS*(pos_enc-pos_enc_old) ,3000);
	pos_enc_old=pos_enc;
	servo_choosen->writeMicroseconds(Micros);

	if(Micros!=memMicros)
	{
//		char affich[16];
//		snprintf(affich,17,"delay= %dus",Micros);
//		afficher(affich);
		memMicros=Micros;
		afficher("delay= %dus",Micros);
	}

	if(!digitalRead(RETOUR))
	{
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sChoice_servo);
	}
    return NULL;
}
void initservo_micros(sState *prev){
	int micros=servo_choosen->readMicroseconds();
	int value_enc=abs(micros-500)*2/PRECISION_MICROS;
	myEnc.write(value_enc);
}
void deinitservo_micros(sState *next){
}

sState sservo_micros={
    0,
    &initservo_micros,
    &deinitservo_micros,
    &testservo_micros
};


