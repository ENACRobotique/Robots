/*
 * blink.cpp
 *
 *  Created on: 4 juin 2013
 *      Author: robot
 */


#include <Arduino.h>
#include <messages.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
//#include "network_cfg.h"

#include "../../../static/communication/network_tools/bn_debug.h"
#include "../../../static/communication/botNet/shared/botNet_core.h"
#include "../../../static/communication/botNet/shared/message_header.h"
extern "C" {
#include "roles.h"
}

typedef struct {
        eServos id;
        int pca_id;
        float a; //director coeff. for degrees to us conversion (associated to each physical servo)   us = a * deg + b
        float b; //additive coeff.
} sServoData;
sServoData servosTable[] = {
        {SERVO_PRIM_GLASS1_HOLD,     0, 10, 520},
        {SERVO_PRIM_GLASS1_RAISE,    1, 10, 520},
        {SERVO_PRIM_GLASS2_HOLD,     2, 10, 520},
        {SERVO_PRIM_GLASS2_RAISE,    3, 10, 520},
        {SERVO_PRIM_GLASS3_HOLD,     4, 10, 520},
        {SERVO_PRIM_GLASS3_RAISE,    5, 10, 520},
        {SERVO_PRIM_LIFT1_UP,        6, 10, 520},
        {SERVO_PRIM_LIFT1_DOOR,      7, 10, 520},
        {SERVO_PRIM_LIFT1_HOLD,      8, 10, 520},
        {SERVO_PRIM_LIFT2_UP,        9, 10, 520},
        {SERVO_PRIM_LIFT2_DOOR,     10, 10, 520},
        {SERVO_PRIM_LIFT2_HOLD,     11, 10, 520},
        {SERVO_PRIM_CORN1_RAMP,     12, 10, 520},
        {SERVO_PRIM_CORN2_RAMP,     13, 10, 520},
        {SERVO_PRIM_CORN_DOOR,      14, 10, 520}
};
#define NUM_SERVOS (sizeof(servosTable)/sizeof(*servosTable))
#define PIN_DBG_LED (13)
#define PIN_MODE_SWICTH (3)
#define PIN_STARTING_CORD (2)
#define PIN_LED_1 (4)
#define PIN_LED_2 (5)

#define PIN_LIMIT_SWITCH_RIGHT (A2)
#define PIN_LIMIT_SWITCH_LEFT (A3)

Adafruit_PWMServoDriver pwm(0x40);

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50. //the servo command frequency (Hz)

void fctModeSwitch(void);
void fctStartingCord(void);


void setup(){
    attachInterrupt(0, fctStartingCord, CHANGE);
    attachInterrupt(1, fctModeSwitch, CHANGE);

    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_DBG_LED, OUTPUT);

    //init led
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, LOW);

    bn_init();

    // do not call init because I²C has already been initialized in bn_init
    // be sure to call reset and setPWMFreq after bn_init...
    pwm.reset();
    pwm.setPWMFreq(SERVO_FREQ);  // 50Hz

    bn_printDbg("start arduino_io");
}


sMsg inMsg, outMsg;
int ledState = 0, ledState1 = 0, i, j, flagModeSwitch = 0, flagStartingCord = 0, ModeSwicth = 0, StartingCord = 0, Led = 0;
int prevLimitSwitchRight = 0, limitSwitchRight = 0, prevLimitSwitchLeft = 0, limitSwitchLeft = 0;
unsigned long led_prevT = 0, time, timeModeSwitch, timeStartingCord, timeLimitSwitchRight, timeLimitSwitchLeft;

int degreesTo4096th(float degrees, float a, float b);

void loop(){
    int ret;
    time = millis();

    if(bn_receive(&inMsg) > 0){
        switch(inMsg.header.type){
        case E_SERVOS:
            for(i = 0; i < (int)inMsg.payload.servos.nb_servos; i++){
                for(j = 0; j < (int)NUM_SERVOS; j++){
                    if(servosTable[j].id == inMsg.payload.servos.servos[i].id){
                        int servoCmd = degreesTo4096th(inMsg.payload.servos.servos[i].angle, servosTable[j].a, servosTable[j].b);
                        pwm.setPWM(servosTable[j].pca_id, 0, servoCmd);
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

                while( (ret = bn_send(&outMsg)) <= 0);
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

    if (time - led_prevT > 200u) {
        led_prevT = time;

        digitalWrite(PIN_DBG_LED, ledState^=1);
    }

    if( (time -  timeStartingCord > 20) && flagStartingCord){
        StartingCord = digitalRead(PIN_STARTING_CORD);

        outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        outMsg.header.type = E_IHM_STATUS;
        outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        outMsg.payload.ihmStatus.nb_states = 1;
        outMsg.payload.ihmStatus.states[0].id = IHM_STARTING_CORD;
        outMsg.payload.ihmStatus.states[0].state = StartingCord;

        while( (ret = bn_send(&outMsg)) <= 0);

        flagStartingCord = 0;
    }

    if( (time -  timeModeSwitch > 20) && flagModeSwitch){
        ModeSwicth = digitalRead(PIN_MODE_SWICTH);
        digitalWrite(PIN_LED_1, ledState1^=1);
        digitalWrite(PIN_LED_2, !ledState1);

        outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        outMsg.header.type = E_IHM_STATUS;
        outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        outMsg.payload.ihmStatus.nb_states = 1;
        outMsg.payload.ihmStatus.states[0].id = IHM_MODE_SWICTH;
        outMsg.payload.ihmStatus.states[0].state = ModeSwicth;
        while( (ret = bn_send(&outMsg)) <= 0);

        flagModeSwitch = 0;
    }

    if( (time -  timeLimitSwitchRight) > 20 ){
        limitSwitchRight = digitalRead(PIN_LIMIT_SWITCH_RIGHT);

        if(limitSwitchRight != prevLimitSwitchRight){
            prevLimitSwitchRight = limitSwitchRight;

            outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
            outMsg.header.type = E_IHM_STATUS;
            outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
            outMsg.payload.ihmStatus.nb_states = 1;
            outMsg.payload.ihmStatus.states[0].id = IHM_LIMIT_SWITCH_RIGHT;
            outMsg.payload.ihmStatus.states[0].state = limitSwitchRight;
            while( (ret = bn_send(&outMsg)) <= 0);
        }
    }

    if( (time -  timeLimitSwitchLeft) > 20 ){
        limitSwitchLeft = digitalRead(PIN_LIMIT_SWITCH_LEFT);

        if(limitSwitchLeft != prevLimitSwitchLeft){
            prevLimitSwitchLeft = limitSwitchLeft;

            outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
            outMsg.header.type = E_IHM_STATUS;
            outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
            outMsg.payload.ihmStatus.nb_states = 1;
            outMsg.payload.ihmStatus.states[0].id = IHM_LIMIT_SWITCH_LEFT;
            outMsg.payload.ihmStatus.states[0].state = limitSwitchLeft;
            while( (ret = bn_send(&outMsg)) <= 0);
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

int degreesTo4096th(float degrees, float a, float b){
    float fCmdOutOf4096 = SERVO_FREQ*4096.*(a*degrees + b)/1000000.;
    unsigned int cmdOutOf4096 (fCmdOutOf4096+0.5);
    return cmdOutOf4096;
}
