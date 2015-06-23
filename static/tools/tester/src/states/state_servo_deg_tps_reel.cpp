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
#include "state_servo_deg_tps_reel.h"
#include "lib_IHM.h"


sState* testservo_deg_tps_reel(){
	static int pos_enc_old=myEnc.read();

	int pos_enc = myEnc.read();

	if(pos_enc!=pos_enc_old){
		servo_choosen->write(pos_enc);
		afficher("Angle= %d", pos_enc);
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
void initservo_deg_tps_reel(sState *prev){
	int angle=servo_choosen->read();		//work only if the servo was already attach and used.
	myEnc.setLimits(0,180);
	myEnc.write(angle);
	myEnc.setMultiplicators(1,10);
	afficher("Angle= %d", angle);
}

void deinitservo_deg_tps_reel(sState *next){
}

sState sservo_deg_tps_reel={
    0,
    &initservo_deg_tps_reel,
    &deinitservo_deg_tps_reel,
    &testservo_deg_tps_reel
};


