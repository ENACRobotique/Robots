/*
 * blink.cpp
 *
 *  Created on: 4 juin 2013
 *      Author: robot
 */


#include <Arduino.h>
#include <messages.h>
//#include "network_cfg.h"

#include "../../../static/communication/network_tools/bn_debug.h"
#include "../../../static/communication/botNet/shared/botNet_core.h"
#include "../../../static/communication/botNet/shared/message_header.h"
#include "../../../static/core/arduino/libraries/Servo/Servo.h"

typedef struct {
        Servo fd;
        eServos id;
        int pin;
} sServoData;
sServoData servosTable[] = {
        {Servo(), SERVO_PRIM_DOOR, 10},     //5
        {Servo(), SERVO_PRIM_FIRE1, 8},     //3
        {Servo(), SERVO_PRIM_FIRE2, 9},     //4
        {Servo(), SERVO_PRIM_ARM_LEFT, 6},  //1
        {Servo(), SERVO_PRIM_ARM_RIGHT, 7}, //2
};
#define NUM_SERVOS (sizeof(servosTable)/sizeof(*servosTable))
#define PIN_DBG_LED (13)
#define PIN_MODE_SWICTH (3)
#define PIN_STARTING_CORD (2)
#define PIN_LED_1 (4)
#define PIN_LED_2 (5)

#define PIN_LIMIT_SWITCH_RIGHT (A2)
#define PIN_LIMIT_SWITCH_LEFT (A3)

void fctModeSwitch(void);
void fctStartingCord(void);


void setup(){
    int i;

    pinMode(PIN_DBG_LED, OUTPUT);
    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_LIMIT_SWITCH_RIGHT, INPUT);
    pinMode(PIN_LIMIT_SWITCH_LEFT, INPUT);

    attachInterrupt(0, fctStartingCord, CHANGE);
    attachInterrupt(1, fctModeSwitch, CHANGE);

    //init led
    digitalWrite(PIN_LED_1, LOW);
    digitalWrite(PIN_LED_2, LOW);

    bn_init();

    bn_printDbg("start arduino_bn_template");

    for(i = 0; i < (int)NUM_SERVOS; i++){
        servosTable[i].fd.attach(servosTable[i].pin);
    }
}

sMsg inMsg, outMsg;
int ledState = 0, i, j, flagModeSwitch = 0, flagStartingCord = 0, ModeSwicth = 0, StartingCord = 0, Led = 0;
int prevLimitSwitchRight = 0, limitSwitchRight = 0, prevLimitSwitchLeft = 0, limitSwitchLeft = 0;
unsigned long led_prevT = 0, time, timeModeSwitch, timeStartingCord, timeLimitSwitchRight, timeLimitSwitchLeft;

void loop(){
    time = millis();



    if(bn_receive(&inMsg) > 0){
        switch(inMsg.header.type){
        case E_SERVOS:
            for(i = 0; i < (int)inMsg.payload.servos.nb_servos; i++){
                for(j = 0; j < (int)NUM_SERVOS; j++){
                    if(servosTable[j].id == inMsg.payload.servos.servos[i].id){
                        servosTable[j].fd.writeMicroseconds(inMsg.payload.servos.servos[i].us);
                    }
                }
            }
            break;
        case E_IHM_STATUS:
            if(inMsg.payload.ihmStatus.nb_states == 0){
                outMsg.header.destAddr = inMsg.header.srcAddr;
                outMsg.header.type = E_IHM_STATUS;
                outMsg.header.size = 2 + 3*sizeof(*outMsg.payload.ihmStatus.states);

                outMsg.payload.ihmStatus.nb_states = 3;

                outMsg.payload.ihmStatus.states[0].id = IHM_STARTING_CORD;
                outMsg.payload.ihmStatus.states[0].state = StartingCord;

                outMsg.payload.ihmStatus.states[1].id = IHM_MODE_SWICTH;
                outMsg.payload.ihmStatus.states[1].state = ModeSwicth;

                outMsg.payload.ihmStatus.states[2].id = IHM_LED;
                outMsg.payload.ihmStatus.states[2].state = Led;

                bn_send(&outMsg);
            }
            else{
                for(i = 0; i < (int)inMsg.payload.ihmStatus.nb_states; i++){
                    switch(inMsg.payload.ihmStatus.states[i].id){
                    case IHM_LED:
                        switch(inMsg.payload.ihmStatus.states[i].state){
                        case 0:
                            digitalWrite(PIN_LED_1, LOW);
                            digitalWrite(PIN_LED_2, LOW);
                            Led = 0;
                            break;
                        case 1:
                            digitalWrite(PIN_LED_1, HIGH);
                            digitalWrite(PIN_LED_2, LOW);
                            Led = 1;
                            break;
                        case 2:
                            digitalWrite(PIN_LED_1, LOW);
                            digitalWrite(PIN_LED_2, HIGH);
                            Led = 2;
                            break;
                        }
                        break;
                    case IHM_STARTING_CORD:
                    case IHM_MODE_SWICTH:
                    default:
                        break;
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

    if( (time -  timeStartingCord > 20) && flagStartingCord){
        StartingCord = digitalRead(PIN_STARTING_CORD);

        outMsg.header.destAddr = role_get_addr(ROLE_IA);
        outMsg.header.type = E_IHM_STATUS;
        outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        outMsg.payload.ihmStatus.nb_states = 1;
        outMsg.payload.ihmStatus.states[0].id = IHM_STARTING_CORD;
        outMsg.payload.ihmStatus.states[0].state = StartingCord;
        bn_send(&outMsg);

        flagStartingCord = 0;
    }

    if( (time -  timeModeSwitch > 20) && flagModeSwitch){
        ModeSwicth = digitalRead(PIN_MODE_SWICTH);

        outMsg.header.destAddr = role_get_addr(ROLE_IA);
        outMsg.header.type = E_IHM_STATUS;
        outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        outMsg.payload.ihmStatus.nb_states = 1;
        outMsg.payload.ihmStatus.states[0].id = IHM_MODE_SWICTH;
        outMsg.payload.ihmStatus.states[0].state = ModeSwicth;
        bn_send(&outMsg);

        flagModeSwitch = 0;
    }

    if( (time -  timeLimitSwitchRight) > 20 ){
        limitSwitchRight = digitalRead(PIN_MODE_SWICTH);

        if(limitSwitchRight != prevLimitSwitchRight){
            prevLimitSwitchRight = limitSwitchRight;

            outMsg.header.destAddr = role_get_addr(ROLE_IA);
            outMsg.header.type = E_IHM_STATUS;
            outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
            outMsg.payload.ihmStatus.nb_states = 1;
            outMsg.payload.ihmStatus.states[0].id = IHM_LIMIT_SWITCH_RIGHT;
            outMsg.payload.ihmStatus.states[0].state = limitSwitchRight;
            bn_send(&outMsg);
        }
    }

    if( (time -  timeLimitSwitchLeft) > 20 ){
        limitSwitchLeft = digitalRead(PIN_MODE_SWICTH);

        if(limitSwitchLeft != prevLimitSwitchLeft){
            prevLimitSwitchLeft = limitSwitchLeft;

            outMsg.header.destAddr = role_get_addr(ROLE_IA);
            outMsg.header.type = E_IHM_STATUS;
            outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
            outMsg.payload.ihmStatus.nb_states = 1;
            outMsg.payload.ihmStatus.states[0].id = IHM_LIMIT_SWITCH_LEFT;
            outMsg.payload.ihmStatus.states[0].state = limitSwitchLeft;
            bn_send(&outMsg);
        }
    }
}

void fctModeSwitch(void){
    timeModeSwitch = millis();
    flagModeSwitch = 1;
}

void fctStartingCord(void){
    timeStartingCord = millis();
    flagStartingCord = 1;
}
