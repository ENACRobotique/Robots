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
#include "states/state_blink.h"
#include "states/state_Menu_principal.h"
#include "states/state_Menu_servo.h"
//#include "states/state_servo_selecter1.h"
#include "states/state_pwm.h"
#include "Arduino.h"
#include "Encoder.h"
#include "Servo.h"

#define NB_menu_servo 3
int retour=0;
int memenc=0;
int deltaenc;
sState *current = &sInitHard;
unsigned long _matchStart;

void setup() {
#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("start");
	Serial.println(digitalRead(PIN_COLOR));
#endif

	attachInterrupt(0, ret, FALLING);
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


#ifdef DEBUG
	static unsigned long prevBlink = millis();
	static char ledState = 0;
	if ((millis() - prevBlink) > 500) {
		ledState ^= 1;
		digitalWrite(PIN_LED, ledState);
		prevBlink = millis();
	}
#endif

	if (current->flag & BIT(E_BLINK))
		blink();

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
