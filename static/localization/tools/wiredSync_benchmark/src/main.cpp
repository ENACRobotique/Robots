/*
 * main.cpp
 *
 *  Created on: 19 janv. 2014
 *      Author: quentin
 */


#define PIN_DBG_LED 13


#include "Arduino.h"
#include "timeout.h"
#include "data.h"

void setup() {
    pinMode(13,OUTPUT);

    Serial.begin(115200);

}

int led=0;
unsigned long int sw=0;
void loop() {

    if ( millis()-sw > 1000){
        led^=1;
        digitalWrite(13,led);
        sw=millis();
    }

    uint32_t sw=0;
    stopwatch(&sw);
    unsigned int i=0;
    for (i=0; i<sizeof(measures)/sizeof(wsMeasure_t); i++){
        wiredSync_intermediateCompute(i*1000000+200000,measures[i].date);
    }
    uint32_t duration1=stopwatch(&sw);
    sw = 0;
    stopwatch(&sw);
    wiredSync_finalCompute(0);
    uint32_t duration2=stopwatch(&sw);
    Serial.print(duration1);
    Serial.print(" ");
    Serial.println(duration2);

}

