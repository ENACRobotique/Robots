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

#include "bn_debug.h"
#include "botNet_core.h"
#include "message_header.h"
extern "C" {
#include "roles.h"
}

typedef struct {;
        int id;
        float a; //director coeff. for degrees to us conversion (associated to each physical servo)   us = a * deg + b
        float b; //additive coeff.
} sServoData;
sServoData servosTable[] = { //servo number (club)|a|b   (us = a*deg+b)
        {1, 10, 520},
        {2, 10, 520},
        {3, 10.033, 526},
        {4, 9.833, 615},
        {5, 10, 520},
        {6, 10, 520},
        {7, 10, 520},
        {8, 10, 520},
        {9, 10, 520},
        {10, 10, 520},
        {11, 10, 520},
        {12, 10, 520},
        {13, 10, 520},
        {14, 10, 520}
};
#define NUM_SERVOS (sizeof(servosTable)/sizeof(*servosTable))
#define PIN_DBG_LED (13)
#define PIN_MODE_SWITCH (3)
#define PIN_STARTING_CORD (2)
#define PIN_LED_BLUE (9)
#define PIN_LED_RED (5)
#define PIN_LED_GREEN (6)

#define PIN_LIMIT_SWITCH_RIGHT (A2)
#define PIN_LIMIT_SWITCH_LEFT (A3)

Adafruit_PWMServoDriver pwm(0x40);

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50. //the servo command frequency (Hz)

#define DEBUG

void fctModeSwitch(void);
void fctStartingCord(void);


void setup(){
    attachInterrupt(0, fctStartingCord, CHANGE);
    attachInterrupt(1, fctModeSwitch, CHANGE);

    pinMode(PIN_LED_BLUE, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_DBG_LED, OUTPUT);

    //init led
    digitalWrite(PIN_LED_BLUE, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);

    bn_init();

    // do not call init because I²C has already been initialized in bn_init
    // be sure to call reset and setPWMFreq after bn_init...
    pwm.reset();
    pwm.setPWMFreq(SERVO_FREQ);  // 50Hz

    bn_printDbg("start arduino_io");
}


sMsg inMsg, outMsg;
int ledState = 0, ledState1 = 0, i, j, flagModeSwitch = 0, flagStartingCord = 0, ModeSwitch = 0, StartingCord = 0, Led = 0;
int prevLimitSwitchRight = 0, limitSwitchRight = 0, prevLimitSwitchLeft = 0, limitSwitchLeft = 0, debounceModeSwitch=0;
int debounceStartingCord;
unsigned long led_prevT = 0, time, timeModeSwitch, timeStartingCord, timeLimitSwitchRight, timeLimitSwitchLeft,timeLedStart;
unsigned int numberLedRepetitions,ledBlinkTimes, durationLedColorCurrent, durationLedColorNext;
sRGB currentLedColor, nextLedColor;


void setLedRGB(unsigned int red, unsigned int green, unsigned int blue);
int degreesTo4096th(float degrees, float a, float b);

void loop(){
    int ret;
    time = millis();

    if(bn_receive(&inMsg) > 0){
        switch(inMsg.header.type){
        case E_SERVOS:
            for(i = 0; i < (int)inMsg.payload.servos.nb_servos; i++){
                for(j = 0; j < (int)NUM_SERVOS; j++){
                    if(servosTable[j].id == inMsg.payload.servos.servos[i].club_id){
                        int servoCmd = degreesTo4096th(inMsg.payload.servos.servos[i].angle, servosTable[j].a, servosTable[j].b);
                        pwm.setPWM(inMsg.payload.servos.servos[i].hw_id, 0, servoCmd);
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
                outMsg.payload.ihmStatus.states[0].state.state_cord = eIhmCord(StartingCord);

                outMsg.payload.ihmStatus.states[1].id = IHM_MODE_SWITCH;
                outMsg.payload.ihmStatus.states[1].state.state_switch = eIhmSwitch(ModeSwitch);

                outMsg.payload.ihmStatus.states[2].id = IHM_LED;
                outMsg.payload.ihmStatus.states[2].state.color1 = currentLedColor;
                outMsg.payload.ihmStatus.states[2].state.color2 = nextLedColor;
                outMsg.payload.ihmStatus.states[2].state.time1 = durationLedColorCurrent;
                outMsg.payload.ihmStatus.states[2].state.time2 = durationLedColorNext;
                outMsg.payload.ihmStatus.states[2].state.nb = ledBlinkTimes;

                while( (ret = bn_send(&outMsg)) <= 0);
            }
            else{
                for(i = 0; i < (int)inMsg.payload.ihmStatus.nb_states; i++){
                    switch(inMsg.payload.ihmStatus.states[i].id){
                    case IHM_LED:

                    	currentLedColor = inMsg.payload.ihmStatus.states[i].state.color1;
                    	nextLedColor = inMsg.payload.ihmStatus.states[i].state.color2;
                    	durationLedColorCurrent = inMsg.payload.ihmStatus.states[i].state.time1;
                    	durationLedColorNext = inMsg.payload.ihmStatus.states[i].state.time2;
                    	numberLedRepetitions = inMsg.payload.ihmStatus.states[i].state.nb;
                    	ledBlinkTimes = 0;

                    	setLedRGB(currentLedColor.red, currentLedColor.green, currentLedColor.blue);
                    	timeLedStart = millis();

                        break;
                    case IHM_STARTING_CORD:
                    case IHM_MODE_SWITCH:
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

    if (ledBlinkTimes < 2*numberLedRepetitions || !numberLedRepetitions){
    	if (time - timeLedStart > durationLedColorCurrent && durationLedColorNext != 0){
    		sRGB colortemp = currentLedColor;
    		currentLedColor = nextLedColor;
    		nextLedColor = colortemp;

    		unsigned int durationtemp = durationLedColorCurrent;
    		durationLedColorCurrent = durationLedColorNext;
    		durationLedColorNext = durationtemp;

    		timeLedStart = time;

    		ledBlinkTimes++;

    		setLedRGB(currentLedColor.red, currentLedColor.green, currentLedColor.blue);
    	}
    }
    else{
    	setLedRGB(0,0,0);
    }

    if (time - led_prevT > 200u) {
        led_prevT = time;

        digitalWrite(PIN_DBG_LED, ledState^=1);
    }

    if( (time -  timeStartingCord > 40) && flagStartingCord){
        StartingCord = digitalRead(PIN_STARTING_CORD);

        if (StartingCord == debounceStartingCord){
        	outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        	outMsg.header.type = E_IHM_STATUS;
        	outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        	outMsg.payload.ihmStatus.nb_states = 1;
        	outMsg.payload.ihmStatus.states[0].id = IHM_STARTING_CORD;
        	outMsg.payload.ihmStatus.states[0].state.state_cord = eIhmCord(StartingCord);

        	while( (ret = bn_send(&outMsg)) <= 0);
        }
        flagStartingCord = 0;
    }

    if( (time -  timeModeSwitch > 40) && flagModeSwitch){
        ModeSwitch = digitalRead(PIN_MODE_SWITCH);

        if (ModeSwitch == debounceModeSwitch){
        	outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        	outMsg.header.type = E_IHM_STATUS;
        	outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        	outMsg.payload.ihmStatus.nb_states = 1;
        	outMsg.payload.ihmStatus.states[0].id = IHM_MODE_SWITCH;
        	outMsg.payload.ihmStatus.states[0].state.state_switch = eIhmSwitch(ModeSwitch);
        	while( (ret = bn_send(&outMsg)) <= 0);
        }

        flagModeSwitch = 0;
    }

//    if( (time -  timeLimitSwitchRight) > 20 ){
//        limitSwitchRight = digitalRead(PIN_LIMIT_SWITCH_RIGHT);
//
//        if(limitSwitchRight != prevLimitSwitchRight){
//            prevLimitSwitchRight = limitSwitchRight;
//
//            outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
//            outMsg.header.type = E_IHM_STATUS;
//            outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
//            outMsg.payload.ihmStatus.nb_states = 1;
//            outMsg.payload.ihmStatus.states[0].id = IHM_LIMIT_SWITCH_RIGHT;
//            outMsg.payload.ihmStatus.states[0].state = limitSwitchRight;
//            while( (ret = bn_send(&outMsg)) <= 0);
//        }
//    }

//    if( (time -  timeLimitSwitchLeft) > 20 ){
//        limitSwitchLeft = digitalRead(PIN_LIMIT_SWITCH_LEFT);
//
//        if(limitSwitchLeft != prevLimitSwitchLeft){
//            prevLimitSwitchLeft = limitSwitchLeft;
//
//            outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
//            outMsg.header.type = E_IHM_STATUS;
//            outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
//            outMsg.payload.ihmStatus.nb_states = 1;
//            outMsg.payload.ihmStatus.states[0].id = IHM_LIMIT_SWITCH_LEFT;
//            outMsg.payload.ihmStatus.states[0].state = limitSwitchLeft;
//            while( (ret = bn_send(&outMsg)) <= 0);
//        }
//    }
}

void fctModeSwitch(void){
    timeModeSwitch = millis();
    debounceModeSwitch = digitalRead(PIN_MODE_SWITCH);
    flagModeSwitch = 1;
}

void fctStartingCord(void){
    timeStartingCord = millis();
    debounceStartingCord = digitalRead(PIN_STARTING_CORD);
    flagStartingCord = 1;
}

int degreesTo4096th(float degrees, float a, float b){
    float fCmdOutOf4096 = SERVO_FREQ*4096.*(a*degrees + b)/1000000.;
    unsigned int cmdOutOf4096 (fCmdOutOf4096+0.5);
    return cmdOutOf4096;
}

void setLedRGB(unsigned int red, unsigned int green, unsigned int blue){
#ifdef DEBUG
	Serial.println(red);
	Serial.print(green);
	Serial.print(blue);
#endif
	analogWrite(PIN_LED_RED, red);
	analogWrite(PIN_LED_GREEN, green);
	analogWrite(PIN_LED_BLUE, blue);
}
