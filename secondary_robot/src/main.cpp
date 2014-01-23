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
#include "lib_radar2.h"
#include "lib_motor.h"
#include "lib_wall.h"
#include "lib_line.h"


Servo launcherServoDown,launcherServoUp, launcherServoNet;

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
    current->init(0);
    }

}


void loop(){

#ifdef DEBUG
    static unsigned long prevBlink=millis();
    static char ledState=0;
    if ( (millis()-prevBlink)>500){
        ledState^=1;
        digitalWrite(PIN_LED,ledState);
        prevBlink=millis();
    }
#endif

    if (current->flag & BIT(E_BLINK) ) blink();
    if (current->flag & BIT(E_WALL)  ) periodicWall();
    if (current->flag & BIT(E_RADAR) ) radarRefresh();
    if (current->flag & BIT(E_MOTOR) ) motAsserTemp(); //motAsser() en attendant le nouveau codeur
    if (current->flag & BIT(E_LINE)  ) asserLine();

    sState *next;
    if (current->testFunction){
        if ( (next=(current->testFunction()) ) ) {
            if (current->deinit) current->deinit(next); //we call deinit of the current state with the pointer to next state
            if (next->init) next->init(current); //we call init of the next state with the pointer of current state
            current=next; //we set the new state
        }
    }

}


