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
#include "state_servo_micros.h"
#include "state_blink.h"


sState* testservo_micros(){
	static int memMicros=500;
	static long temps_enc=0;
	static int pos_enc_old=0;

	int pos_enc=abs(myEnc.read())/2;
	int delta=PRECISION_MICROS;

	if(pos_enc!=pos_enc_old)
	{
		long deltat=millis()-temps_enc;
		if(deltat<DUREE_BIG_STEPS)
		{
			delta=10*PRECISION_MICROS;
		}
		temps_enc=millis();
	}
	int Micros = max((memMicros-500+delta*(pos_enc-pos_enc_old))%3000+500,500);
	Micros=min(Micros,3000);
	pos_enc_old=pos_enc;
	servotest.writeMicroseconds(Micros);

	if(Micros!=memMicros)
	{
		char affich[16];
		snprintf(affich,17,"delay= %dus",Micros);
		afficher(affich);
		memMicros=Micros;
	}

	if(!digitalRead(RETOUR))
	{
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_servo);
	}
    return NULL;
}
void initservo_micros(sState *prev){
	servotest.attach(PIN_PWM_SERVO);
	int micros=servotest.readMicroseconds();
	int value_enc=abs(micros-500)*2/PRECISION_MICROS;
	myEnc.write(value_enc);

}
void deinitservo_micros(sState *next){
	servotest.detach();
}

void servo_micros(){

}

sState sservo_micros={
    0,
    &initservo_micros,
    &deinitservo_micros,
    &testservo_micros
};


