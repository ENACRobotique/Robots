/*
 * state-blink.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include <Arduino.h>
#include <params.h>
#include <state_types.h>
#include <stddef.h>
#include <tools.h>
#include "state_blink.h"

#include "../../../../core/arduino/libraries/LiquidCrystal/LiquidCrystal.h"
LiquidCrystal lcd(6, 7, 8, 10, 11, 12);

sState* testBlink(){
    return NULL;
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
        prevMillis+=100;
    }




}

sState sBlink={
    BIT(E_BLINK),
    &initBlink,
    &deinitBlink,
    &testBlink
};






void afficher(const char * chaine)
{
	  lcd.clear();
	  lcd.home();
	  lcd.write(chaine);
}

