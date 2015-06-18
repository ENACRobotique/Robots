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

#include "state_Menu_pwm.h"
#include "state_pwm_0_255.h"
#include "lib_IHM.h"
#include "state_choice_servo.h"

sState* testpwm_0_255(){
	static long temps_enc=0;
	static int pos_enc_old=myEnc.read();

	int pos_enc = myEnc.read();

	if(pos_enc!=pos_enc_old){
		if(millis()-temps_enc < DUREE_BIG_STEPS)
		{
			pos_enc = pos_enc_old + 10 * (pos_enc - pos_enc_old);
		}
		pos_enc = CLAMP(0, pos_enc, 255);
		temps_enc=millis();

		display(pos_enc);
		myEnc.write(pos_enc);
		pos_enc_old=pos_enc;
		analogWrite(PIN_PWM_SERVO,pos_enc);
	}

//	if(!digitalRead(SELECT)){	//nécessite de valider avant que le servo ne se déplace
//		while(!digitalRead(SELECT));
//		analogWrite(PIN_PWM_SERVO,pos_enc);
//	}

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_pwm);
	}
    return NULL;
}

void initpwm_0_255(sState *prev){
	if(servo3.attached()){
		servo3.detach();
	}
	pinMode(PIN_PWM_SERVO,OUTPUT);
	myEnc.write(0);
	analogWrite(PIN_PWM_SERVO,0);
	display(0);
}

void deinitpwm_0_255(sState *next){
}

void display(int enc){
	if(mode_pwm == PWM_255){
		afficher("D.C.= %d /255", enc);
	}

	if(mode_pwm == PWM_5V){
		int unit = (5*enc)/255;
		int dec = (50*enc)/255 - 10 * unit;
		afficher("U = %d,%d / 5 V", unit,dec);
	}

	if(mode_pwm == PWM_PC){
		int pc = (100*enc)/255;
		afficher("D.C. = %d %%", pc);
	}
}

sState spwm_0_255={
    0,
    &initpwm_0_255,
    &deinitpwm_0_255,
    &testpwm_0_255
};
