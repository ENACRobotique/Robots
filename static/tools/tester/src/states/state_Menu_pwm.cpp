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
#include "state_pwm_pc.h"
#include "state_pwm_0_5.h"
#define NB_menu_pwm 3

const char *menu_pwm[] = {
		  "  0 - 255",
		  " 0 - 100 %",
		  "  0 - 15 V",
		};

sState* testMenu_pwm(){
	static int memPosition;
	int Position = myEnc.read();    //position du selecteur modulo le nombre de choix possible

	if(Position != memPosition){  //on affiche que si on change de position
		if (Position != CLAMP(0,Position,NB_menu_pwm)){		//on ne descend pas dans les négatifs
			Position = CLAMP(0,Position,NB_menu_pwm);
			myEnc.write(Position);
		}
		afficher(menu_pwm[Position]);
	    memPosition=Position;
	}

	if(!digitalRead(SELECT)){	//si on appui sur select:
		while(!digitalRead(SELECT));		//on attend qu'on relache

		switch (Position)		//et on va dans le bon état
		{
			case 0:{ return(&spwm_0_255); break; }
			case 1:{ return(&spwm_pc); break; }
			case 2:{ return(&spwm_0_5); break; }
		  }
	  }

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_principal);
	}

return NULL;
}

void initMenu_pwm(sState *prev){
			myEnc.write(0);
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
