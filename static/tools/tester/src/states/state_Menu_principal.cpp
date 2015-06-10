/*
 * state-Menu_principal.cpp
 *
 *  Created on: 2015
 *      Author: Fab
 */

#include <Arduino.h>
#include <Encoder.h>
#include <params.h>
#include <lib_IHM.h>
#include <state_mode_servo.h>
#include <state_Menu_pwm.h>
#include <state_types.h>
#include <stddef.h>
#include "state_Menu_principal.h"

#include "../../../../core/arduino/libraries/LiquidCrystal/LiquidCrystal.h"

#define NB_menu_principal 5
const char *menu_principal[] = {
		  "SERVOS",
		  "PWM",
		  "I2C",
		  "CODEUR",
		  "Analog Read"
		};

sState* testMenu_principal(){
		static int memPosition;
		int Position = (myEnc.read()/2)%NB_menu_principal;    //position du selecteur modulo le nombre de choix possible
		if (Position < 0){		//on ne descend pas dans les négatifs
			Position=0;
			myEnc.write(0);
		}

		   if(Position != memPosition)  //on affiche que si on change de position
		   {
		      afficher(menu_principal[Position]);
		      memPosition=Position;
		   }

		  if(!digitalRead(SELECT))	//si on appui sur select:
		  {
			  while(!digitalRead(SELECT));		//on attend qu'on relache

		    switch (Position)		//et on va dans le bon état
		    {
		        case 0:{ return(&smode_servo); break; }
		        case 1:{ return(&sMenu_pwm); break; }
	//	        case 2:{ i2c(); break; }
	//	        case 3:{ liaison_serie(); break; }
	//	        case 4:{ analogread(); break;}
		        //default:
		     }
		  }

    return NULL;
}

void initMenu_principal(sState *prev){
	myEnc.write(0);
	afficher(menu_principal[0]);

}

void deinitMenu_principal(sState *next){
}


sState sMenu_principal={
    0,
    &initMenu_principal,
    &deinitMenu_principal,
    &testMenu_principal
};
