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

#include "state_Menu_principal.h"
#include "state_Menu_pwm.h"
#include "lib_IHM.h"
#include "state_pwm_0_255.h"

#define NB_menu_pwm 3
int mode_pwm = 0;
const char *menu_pwm[] = {
		  "  0 - 255",
		  "  0 - 100 %",
		  "  0 - 5 V",
		};

sState* testMenu_pwm(){
	static int memPosition;
	int Position = myEnc.read();    //position du selecteur modulo le nombre de choix possible

	if(Position != memPosition){  //on affiche que si on change de position
		afficher(menu_pwm[Position]);
	    memPosition=Position;
	}

	if(!digitalRead(SELECT)){	//si on appui sur select:
		while(!digitalRead(SELECT));		//on attend qu'on relache

		switch (Position)		//et on va dans le bon état
		{
			case 0:{ mode_pwm = PWM_255; break; }
			case 1:{ mode_pwm = PWM_PC; break; }
			case 2:{ mode_pwm = PWM_5V; break; }
		}
		return &spwm_0_255;
	}

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_principal);
	}

return NULL;
}

void initMenu_pwm(sState *prev){
			myEnc.setLimits(0,NB_menu_pwm-1);
			myEnc.write(0);
			myEnc.setMultiplicators(1,1);
			afficher(menu_pwm[0]);
}
void deinitMenu_pwm(sState *next){
}

sState sMenu_pwm={
    0,
    &initMenu_pwm,
    &deinitMenu_pwm,
    &testMenu_pwm
};
