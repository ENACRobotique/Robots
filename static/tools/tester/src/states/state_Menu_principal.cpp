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
#include "state_blink.h"
#include "state_servo_selecter1.h"
#include "state_servo_selecter2.h"
#include "state_pwm.h"
#define NB_menu_principal 4



Encoder myEnc(3, 4);


sState* testMenu_principal(){
	const char *menu_principal[] = {
		  "SERVOS",
		  "PWM",
		  "I2C",
		  "LIAISON SERIE",
		};


		static int memPosition;
		int Position = (abs(myEnc.read())/2)%NB_menu_principal;    //position du selecteur

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
	//	        case 2:{ i2c(); break; }
	//	        case 3:{ liaison_serie(); break; }
		        //default:
		     }
		  }


    return NULL;
}
void initMenu_principal(sState *prev){
	afficher("SERVOS");

}
void deinitMenu_principal(sState *next){

}


sState sMenu_principal={
    0,
    &initMenu_principal,
    &deinitMenu_principal,
    &testMenu_principal
};
