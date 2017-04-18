/*
 * main.cpp
 *
 *  Created on: 19 janv. 2014
 *      Author: quentin
 */


#define PIN_DBG_LED 13


#include "Arduino.h"

void setup() {
    pinMode(13,OUTPUT);
}

int led=0;
unsigned long int sw=0;
void loop() {

    if ( millis()-sw > 1000){
        led^=1;
        digitalWrite(13,led);
        sw=millis();
    }

}

