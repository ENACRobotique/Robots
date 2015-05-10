/*
 * notmain.cpp
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#include "params.h"
#include "tools.h"
#include "states/state_types.h"
#include "states/state_hardinit.h"
#include "libs/lib_IHM.h"
#include "states/state_Menu_principal.h"
#include "states/state_Menu_servo.h"
#include "states/state_Menu_pwm.h"
#include "Arduino.h"
#include "Encoder.h"
#include "Servo.h"

#define NB_menu_servo 3
sState *current = &sInitHard;
unsigned long _matchStart;

void setup() {
#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("start");
#endif

	digitalWrite(2, HIGH);


	if (current->init) {

#ifdef DEBUG
		Serial.println("sortie init mat");
#endif
		current->init(0);
	}

	pinMode(SELECT,INPUT_PULLUP);
	pinMode(RETOUR,INPUT_PULLUP);
}

void loop() {

	sState *next;
	if (current->testFunction) {
		if ((next = (current->testFunction()))) {
			if (current->deinit)
				current->deinit(next); //we call deinit of the current state with the pointer to next state
			if (next->init)
				next->init(current); //we call init of the next state with the pointer of current state
			current = next; //we set the new state
		}
	}

}
