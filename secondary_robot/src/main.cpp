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
#include "Arduino.h"
#include "lib_radar.h"
#include "lib_motor.h"
#include "lib_wall.h"
#include "lib_line.h"
#include "lib_attitude.h"
#include "lib_heading.h"
#include "state_wait.h"


sState *current=&sInitHard;

unsigned long _matchStart;

void setup(){
#ifdef DEBUG
    Serial.begin(115200);
    Serial.println("start");
    Serial.println(digitalRead(PIN_COLOR));
#endif


    if (current->init) {

#ifdef DEBUG
    Serial.println("sortie init mat");
#endif
    current->init(NULL);
    }

}


void loop(){

#ifdef DEBUG
    static unsigned long prevBlink=millis();
    static char ledState=0;
    if ( (millis()-prevBlink)>500){
        ledState^=1;
        //digitalWrite(PIN_LED,ledState);
        prevBlink=millis();
    }
#endif

    if (current->flag & BIT(E_BLINK) ) blink();
    //if (current->flag & BIT(E_WALL)  ) periodicWall();
    if (current->flag & BIT(E_RADAR) ) radarRefresh();
    if (current->flag & BIT(E_MOTOR) ) motAsser();
    if (current->flag & BIT(E_LINE) )  asserLine();
#ifdef ATTITUDE
    if (current->flag & BIT(E_ATTITUDE) )  attitudeAsser();
#endif
#ifdef HEADING
    if (current->flag & BIT(E_HEADING) )  headingAsser();
#endif

    sState *next;
    if (current->testFunction){
        if ( (next=(current->testFunction()) ) ) {	// we call the current test function, wich return the next state, or NULL to continue in the same state
            if (current->deinit) current->deinit(next); //we call deinit of the current state with the pointer to next state
            if (next->init) next->init(current); //we call init of the next state with the pointer of current state
            current=next; //we set the new state
        }
        if((millis() - _matchStart) > TIME_MATCH_STOP){
        	next = &sWait;	//stop the match
        	if (current->deinit) current->deinit(next); //we call deinit of the current state with the pointer to next state
        	if (next->init) next->init(current); //we call init of the next state with the pointer of current state
        	current=next; //we set the new state
        }
    }

}


