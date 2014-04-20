/*
 * blink.cpp
 *
 *  Created on: 4 juin 2013
 *      Author: robot
 */


#include <Arduino.h>
#include <messages.h>
//#include "network_cfg.h"

#include "../../../static/communication/botNet/shared/bn_debug.h"
#include "../../../static/communication/botNet/shared/botNet_core.h"
#include "../../../static/communication/botNet/shared/message_header.h"
#include "../../../static/core/arduino/libraries/Servo/Servo.h"

typedef struct {
        Servo fd;
        eServos id;
        int pin;
} sServoData;
sServoData servosTable[] = {
        {Servo(), SERVO_PRIM_DOOR, 8}
};
#define NUM_SERVOS (sizeof(servosTable)/sizeof(*servosTable))
#define PIN_DBG_LED (13)

void setup(){
    int i;

    pinMode(PIN_DBG_LED, OUTPUT);

    bn_init();

    bn_printDbg("start arduino_bn_template");

    for(i = 0; i < (int)NUM_SERVOS; i++){
        servosTable[i].fd.attach(servosTable[i].pin);
    }
}

sMsg msg;
int ledState = 0, i, j;
unsigned long led_prevT = 0, time;

void loop(){
    time = millis();

    if(bn_receive(&msg) > 0){
        switch(msg.header.type){
        case E_SERVOS:
            for(i = 0; i < (int)msg.payload.servos.nb_servos; i++){
                for(j = 0; j < (int)NUM_SERVOS; j++){
                    if(servosTable[j].id == msg.payload.servos.servos[i].id){
                        servosTable[j].fd.writeMicroseconds(msg.payload.servos.servos[i].us);
                    }
                }
            }
            break;
        default:
            break;
        }
    }

    if(time - led_prevT > 200){
        led_prevT = time;

        digitalWrite(PIN_DBG_LED, ledState^=1);
    }
}
