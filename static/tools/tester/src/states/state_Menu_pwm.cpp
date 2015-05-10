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
#include "state_Menu_pwm.h"
#include "lib_IHM.h"
#include "state_pwm_0_255.h"
#include "state_pwm_pc.h"
#include "state_pwm_0_5.h"
#define NB_menu_pwm 3

sState* testMenu_pwm(){
	const char *menu_pwm[] = {
		  "  0 - 255",
		  "   %",
		  "  0 - 5 V",
		};


		static int memPosition;
		int Position = (myEnc.read()/2)%NB_menu_pwm;    //position du selecteur
		   if(Position != memPosition)  //on affiche que si on change de position
		   {
		      afficher(menu_pwm[Position]);
		      memPosition=Position;
		   }

		  if(!digitalRead(SELECT))
		  {
			while(!digitalRead(SELECT));
		    switch (Position)
		    {
		        case 0:{ return(&spwm_0_255); break; }
		        case 1:{ return(&spwm_pc); break; }
		        case 2:{ return(&spwm_0_5); break; }
		        //default:
		     }
		  }

		  if(!digitalRead(RETOUR))
		  {
			  delay(DELAY_BOUNCE);	//anti rebond
			  while(!digitalRead(RETOUR));	//attente du relachement du bouton
			  return(&sMenu_principal);
		  }


    return NULL;
}
void initMenu_pwm(sState *prev){
			myEnc.write(0);
			afficher("  0 - 255");
}
void deinitMenu_pwm(sState *next){

}


sState sMenu_pwm={
    0,
    &initMenu_pwm,
    &deinitMenu_pwm,
    &testMenu_pwm
};
