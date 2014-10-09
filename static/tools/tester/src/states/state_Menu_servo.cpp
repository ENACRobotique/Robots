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
#include "state_servo_micros.h"
#define NB_menu_servo 3



Servo servotest;


sState* testMenu_servo(){
	const char *menu_servo[] = {
		  "ANGLE tps reel",
		  "ANGLE validation",
		  "MICROSECONDES",
		};


		static int memPosition;
		int Position = (abs(myEnc.read())/2)%NB_menu_servo;    //position du selecteur

		   if(Position != memPosition)  //on affiche que si on change de position
		   {
		      afficher(menu_servo[Position]);
		      memPosition=Position;
		   }

		  if(!digitalRead(SELECT))
		  {
			while(!digitalRead(SELECT));
		    switch (Position)
		    {
		        case 0:{ return(&sservo_selecter2); break; }
		        case 1:{ return(&sservo_selecter1); break; }
		        case 2:{ return(&sservo_micros); break; }
		        //default:
		     }
		  }

		  if(retour)
		  {
		  	retour=0;
		  	return(&sMenu_principal);
		  }


    return NULL;
}
void initMenu_servo(sState *prev){
	const char *menu_servo[] = {
			  "ANGLE tps reel",
			  "ANGLE validation",
			  "MICROSECONDES",
			};
			int Position = (abs(myEnc.read())/2)%NB_menu_servo;
			afficher(menu_servo[Position]);
}
void deinitMenu_servo(sState *next){

}


sState sMenu_servo={
    0,
    &initMenu_servo,
    &deinitMenu_servo,
    &testMenu_servo
};
