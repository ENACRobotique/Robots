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
#include "state_funny.h"


sState* testBlink(){
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    return 0;
}
void initBlink(sState *prev){
    pinMode(PIN_LED,OUTPUT);

}
void deinitBlink(sState *next){
    digitalWrite(PIN_LED,LOW);
}

void blink(){
    static int state=1;
    static unsigned long prevMillis=millis();
    if ( (millis()-prevMillis) >100){
        digitalWrite(PIN_LED, state);    // set the LED off
        state^=1;
        prevMillis=millis();
    }
}

sState sBlink={
    BIT(E_BLINK),
    &initBlink,
    &deinitBlink,
    &testBlink
};
