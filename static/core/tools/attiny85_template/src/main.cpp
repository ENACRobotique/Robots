/*
 * main.cpp
 *
 *  Created on: dec. 2015
 *      Author: Fabien
 */
#include "Arduino.h"

#define LED 1

void setup() {
    pinMode(LED,OUTPUT);
}

int led=0;
unsigned long int prev_time=0;

void loop() {

    if ( millis()-prev_time > 500){
        led^=1;
        digitalWrite(LED,led);
        prev_time=millis();
    }
}

