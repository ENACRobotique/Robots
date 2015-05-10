/*
 * state_Menu_servo.cpp
 *
 *  Created on: 2015
 *      Author: Fab
 */

#include "Arduino.h"
#include "../params.h"
#include "../tools.h"
#include "state_types.h"

#include "state_Menu_principal.h"
#include "state_Menu_servo.h"
#include "lib_IHM.h"
#include "state_servo_deg_validation.h"
#include "state_servo_deg_tps_reel.h"
#include "state_servo_micros.h"

Servo servotest;

#define NB_menu_servo 3
const char *menu_servo[] = {
	  "ANGLE tps reel",
	  "ANGLE validation",
	  "MICROSECONDES",
	};

sState* testMenu_servo(){
		static int memPosition;
		int Position = (myEnc.read()/2)%NB_menu_servo;    //position du selecteur
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
		        case 0:{ return(&sservo_deg_tps_reel); break; }
		        case 1:{ return(&sservo_deg_validation); break; }
		        case 2:{ return(&sservo_micros); break; }
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
void initMenu_servo(sState *prev){
			myEnc.write(0);
			afficher(menu_servo[0]);
}
void deinitMenu_servo(sState *next){

}


sState sMenu_servo={
    0,
    &initMenu_servo,
    &deinitMenu_servo,
    &testMenu_servo
};
