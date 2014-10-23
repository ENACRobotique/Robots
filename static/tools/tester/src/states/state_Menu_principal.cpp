/*
 * state-blink.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include <Arduino.h>
#include <Encoder.h>
#include <params.h>
#include <state_blink.h>
#include <state_Menu_servo.h>
#include <state_pwm.h>
#include <state_types.h>
#include <stddef.h>
//#include "../tools.h"
#include "state_Menu_principal.h"
#include "state_analogRead.h"

#include "../../../../core/arduino/libraries/LiquidCrystal/LiquidCrystal.h"

#define NB_menu_principal 5

Encoder myEnc(3, 4);


sState* testMenu_principal(){
	const char *menu_principal[] = {
		  "SERVOS",
		  "PWM",
		  "Analog Read",
		  "I2C",
		  "LIAISON SERIE",
		};


		static int memPosition;
		//int Position = (abs(myEnc.read())/2)%NB_menu_principal;    //position du selecteur
		int Position = (myEnc.read()/2)%NB_menu_principal;    //position du selecteur

		   if(Position != memPosition)  //on affiche que si on change de position
		   {
		      afficher(menu_principal[Position]);
		      memPosition=Position;
		   }

		  if(!digitalRead(SELECT))
		  {
			  while(!digitalRead(SELECT));

		    switch (Position)
		    {
		        case 0:{ return(&sMenu_servo); break; }
		        case 1:{ return(&spwm); break; }
		        case 2:{ return(&sanalogRead); break; }
	//	        case 3:{ i2c(); break; }
	//	        case 4:{ liaison_serie(); break; }
		        //default:
		     }
		  }


    return NULL;
}

void initMenu_principal(sState *prev){
	const char *menu_principal[] = {
			  "SERVOS",
			  "PWM",
			  "Analog Read",
			  "I2C",
			  "LIAISON SERIE",
			};
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
