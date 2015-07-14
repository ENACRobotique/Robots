/*
 * state-Menu_principal.cpp
 *
 *  Created on: 2015
 *      Author: Fab
 */

#include <Arduino.h>
#include <params.h>
#include <tools.h>
#include <lib_IHM.h>
#include <state_mode_servo.h>
#include <state_Menu_pwm.h>
#include <state_types.h>
#include <stddef.h>
#include "state_Menu_principal.h"
#include "state_Analog_read.h"
#include "state_Codeur.h"
#include "state_Scan_i2c.h"

int memEncoder=0;
#define NB_menu_principal 5
const char *menu_principal[] = {
		  "SERVOS",
		  "PWM",
		  "CODEUR",
		  "Analog Read",
		  "I2C"
		};

sState* testMenu_principal(){
		static int memPosition;
		int Position = myEnc.read();    //position du selecteur modulo le nombre de choix possible

		if(Position != memPosition){  //on affiche que si on change de position
			afficher(menu_principal[Position]);
		    memPosition=Position;
		}

		if(!digitalRead(SELECT)){	//si on appui sur select:
			while(!digitalRead(SELECT));		//on attend qu'on relache

			switch (Position)		//et on va dans le bon état
			{
				case 0:{ return(&smode_servo); break; }
				case 1:{ return(&sMenu_pwm); break; }
			    case 2:{ return(&sCodeur); break; }
			    case 3:{ return(&sAnalog_read); break;}
			    case 4:{ return(&sScan_i2c); break;}
			  }
		  }

    return NULL;
}

void initMenu_principal(sState *prev){
	myEnc.setLimits(0,NB_menu_principal-1);
	myEnc.write(memEncoder);
	myEnc.setMultiplicators(1,1);
	afficher(menu_principal[memEncoder]);
}

void deinitMenu_principal(sState *next){
	memEncoder = myEnc.read();
}


sState sMenu_principal={
    0,
    &initMenu_principal,
    &deinitMenu_principal,
    &testMenu_principal
};
