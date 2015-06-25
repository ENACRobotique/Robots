/*
 * state_mode_servo.cpp
 *
 *  Created on: 2015
 *      Author: Fab
 */

#include "Arduino.h"
#include "params.h"
#include "tools.h"
#include "state_types.h"

#include "state_Menu_principal.h"
#include "state_mode_servo.h"
#include "state_choice_servo.h"
#include "lib_IHM.h"

int mode_servo=SM_NO_MODE;

#define NB_mode_servo 3
const char *menu_mode[] = {
	  "ANGLE tps reel",
	  "ANGLE validation",
	  "MICROSECONDES",
	};

sState* testmode_servo(){
	static int memPosition;
	int Position = myEnc.read();    //position du selecteur

	if(Position != memPosition){  //on affiche que si on change de position
		afficher(menu_mode[Position]);
		memPosition=Position;
	}

	if(!digitalRead(SELECT)){
		while(!digitalRead(SELECT));
		switch (Position)
		{
			case 0:{ mode_servo = SM_TPS_REEL; break; }
			case 1:{ mode_servo = SM_VALID; break; }
			case 2:{ mode_servo = SM_MICROS; break; }
		}
		return &sChoice_servo;
	}

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_principal);
	}

    return NULL;
}

void initmode_servo(sState *prev){
			myEnc.setLimits(0,NB_mode_servo-1);
			myEnc.write(0);
			myEnc.setMultiplicators(1,1);
			afficher(menu_mode[0]);
}
void deinitmode_servo(sState *next){
}

sState smode_servo={
    0,
    &initmode_servo,
    &deinitmode_servo,
    &testmode_servo
};
