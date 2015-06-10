/*
 * state_choice_servo.cpp
 *
 *  Created on: 2015
 *      Author: Fab
 */

#include "Arduino.h"
#include "../params.h"
#include "../tools.h"
#include "state_types.h"

#include "state_mode_servo.h"
#include "lib_IHM.h"
#include "state_servo_deg_validation.h"
#include "state_servo_deg_tps_reel.h"
#include "state_servo_micros.h"
#include "state_mode_servo.h"

Servo * servo_choosen;
Servo servo1;
Servo servo2;
Servo servo3;


#define NB_Choice_servo 3
const char *Choice_servo[] = {
	  "Servo 1",
	  "Servo 2",
	  "Servo 3",
	};

sState* testChoice_servo(){
		static int memPosition;
		int Position = (myEnc.read()/2)%NB_Choice_servo;    //position du selecteur
		   if(Position != memPosition)  //on affiche que si on change de position
		   {
		      afficher(Choice_servo[Position]);
		      memPosition=Position;
		   }

		  if(!digitalRead(SELECT))
		  {
			while(!digitalRead(SELECT));

			switch (Position)
			{
				case 0:{ servo_choosen = &servo1; break; }
				case 1:{ servo_choosen = &servo2; break; }
				case 2:{ servo_choosen = &servo3; break; }
				//default:
			 }

		    switch (mode_servo)
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
			  return(&smode_servo);
		  }


    return NULL;
}
void initChoice_servo(sState *prev){
			myEnc.write(0);
			afficher(Choice_servo[0]);
}
void deinitChoice_servo(sState *next){

	if(!servo_choosen->attached()){
		if(servo_choosen == &servo1){
			servo_choosen->attach(SERVO1);
		}
		if(servo_choosen == &servo2){
			servo_choosen->attach(SERVO2);
		}
		if(servo_choosen == &servo3){
			servo_choosen->attach(PIN_PWM_SERVO);
		}
	}
}


sState sChoice_servo={
    0,
    &initChoice_servo,
    &deinitChoice_servo,
    &testChoice_servo
};
