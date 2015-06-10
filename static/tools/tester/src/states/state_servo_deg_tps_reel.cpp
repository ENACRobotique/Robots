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

#include "state_choice_servo.h"
#include "state_servo_deg_tps_reel.h"
#include "lib_IHM.h"


sState* testservo_deg_tps_reel(){
	static int memAngle=servo_choosen->read();
	static long temps_enc=0;
	static int pos_enc_old=myEnc.read()/2;

	int pos_enc = myEnc.read()/2;

	if(pos_enc!=pos_enc_old)
	{
		if(millis()-temps_enc<DUREE_BIG_STEPS)
		{
			pos_enc = pos_enc_old + 10 * (pos_enc - pos_enc_old);
			myEnc.write(pos_enc);
		}
		temps_enc=millis();
	}

	pos_enc = CLAMP(0,pos_enc,180);

	if (pos_enc == 0)
	{
		pos_enc=0;
		myEnc.write(0);
	}

	int Angle = CLAMP(0,memAngle+(pos_enc-pos_enc_old),180);
	pos_enc_old=pos_enc;

	servo_choosen->write(Angle);

	if(Angle!=memAngle)
	{
		afficher("Angle= %d", Angle);
		memAngle=Angle;
	}

	if(!digitalRead(RETOUR))
	{
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sChoice_servo);
	}
    return NULL;
}
void initservo_deg_tps_reel(sState *prev){
	int angle=servo_choosen->read();		//work only if the servo was already attach and used.
	int value_enc=angle*2.0/5.0;
	myEnc.write(value_enc);
	//char affich[16];
	//snprintf(affich,17,"Angle = %d",angle);
	//afficher(affich);
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


