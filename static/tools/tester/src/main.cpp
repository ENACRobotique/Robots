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
#include "Arduino.h"

sState *current = &sInitHard;
unsigned long _matchStart;

void setup() {
#ifdef NOLCD
	Serial.begin(115200);
	Serial.println("start");
#endif

	if (current->init) {

#ifdef DEBUG
		Serial.println("sortie init mat");
#endif
		current->init(0);
	}
}

void blink(){
    static unsigned int inv_state=0;
    static unsigned long prevMillis=0;
    if ( (millis()-prevMillis) >(100+900*inv_state)){
        digitalWrite(LED1, inv_state);    // set the LED off
        inv_state^=1;
        prevMillis=millis();
    }
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

	blink();
}
