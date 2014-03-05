/*
 * blink.cpp
 *
 *  Created on: 4 juin 2013
 *      Author: robot
 */


#include "Arduino.h"

#include "messages.h"
#include "network_cfg.h"
#include "../static/communication/botNet/shared/botNet_core.h"
#include "../static/communication/botNet/shared/bn_debug.h"

#define PIN_DBG_LED (13)

void setup(){
    pinMode(PIN_DBG_LED, OUTPUT);

    bn_init();

    bn_printDbg("start arduino_bn_template");
}

sMsg msg;
int ledState = 0;
unsigned long led_prevT = 0, time, blinkPeriod = 200;

void loop(){
    time = millis();

    if(bn_receive(&msg) > 0){
        bn_printfDbg("got message: type %hhu, size %hhu\n", msg.header.type, msg.header.size);

        blinkPeriod = 1000 - blinkPeriod;
    }

    if(time - led_prevT > blinkPeriod){
        led_prevT = time;

        digitalWrite(PIN_DBG_LED, ledState^=1);
    }
}
