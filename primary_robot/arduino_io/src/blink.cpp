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
        {1, -9.5, 2350},
        {2, -9.5, 2350},
        {3, 10.033, 526},
        {4, 9.833, 615},
        {5, 9.289, 619},
        {6, 9.944, 552.5},
        {7, 9.20, 545},
        {8, 9.74, 548},
        {9, 9.59, 554},
        {10, 10.26, 554},
        {11, -9.24, 2373},
        {12, -9.12, 2375},
        {13, -9.50, 2402},
        {14, -9.57, 2366},
		{15, -9.80, 2386},
		{16, -10.268, 2396},
		{17, -9.68, 2400},
		{18, -8.50, 2410},
		{19, 12.15, 883},
		{20, 11.73, 881}
};
#define NUM_SERVOS (sizeof(servosTable)/sizeof(*servosTable))
#define PIN_DBG_LED (13)
#define PIN_MODE_SWITCH (3)
#define PIN_STARTING_CORD (2)
#define PIN_LED_BLUE (9)
#define PIN_LED_RED (5)
#define PIN_LED_GREEN (6)

#define PIN_PRESENCE_1 (4)
#define PIN_PRESENCE_2 (7)
#define PIN_PRESENCE_3 (8)
#define PIN_PRESENCE_4 (10)
#define PIN_PRESENCE_5 (11)

Adafruit_PWMServoDriver pwm(0x40);

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50. //the servo command frequency (Hz)

//#define DEBUG

void fctModeSwitch(void);
void fctStartingCord(void);


void setup(){
    attachInterrupt(0, fctStartingCord, CHANGE);
    attachInterrupt(1, fctModeSwitch, CHANGE);

    pinMode(PIN_LED_BLUE, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_DBG_LED, OUTPUT);
    pinMode(PIN_MODE_SWITCH, INPUT);
    pinMode(PIN_STARTING_CORD, INPUT);

    //init led
    digitalWrite(PIN_LED_BLUE, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);


    bn_attach(E_ROLE_SETUP, role_setup);
    bn_init();

    // do not call init because I²C has already been initialized in bn_init
    // be sure to call reset and setPWMFreq after bn_init...
    pwm.reset();
    pwm.setPWMFreq(SERVO_FREQ);  // 50Hz

#ifdef DEBUG
    bn_printDbg("start arduino_io");
#endif
}


sMsg inMsg, outMsg;
int ledState = 0, ledState1 = 0, i, j, flagModeSwitch = 0, flagStartingCord = 0, ModeSwitch = 0, StartingCord = 0, Led = 0;
int flagPresence1=0, flagPresence2=0, flagPresence3=0, flagPresence4=0, debounceModeSwitch=0;
int debounceStartingCord, presence1Old=0, presence1=0, presence2Old=0, presence2=0, presence3Old=0, presence3=0, presence4Old=0;
int presence4=0, presence5Old=0, presence5=0, flagPresence5=0;
unsigned long led_prevT = 0, time=0, timeModeSwitch=0, timeStartingCord=0, timePresence1=0, timePresence2=0, timePresence3=0,timeLedStart=0;
unsigned long timePresence4=0, timePresence5=0;
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

                outMsg.header.destAddr = inMsg.header.srcAddr;
                outMsg.header.type = E_IHM_STATUS;
                outMsg.header.size = 2 + 3*sizeof(*outMsg.payload.ihmStatus.states);

                outMsg.payload.ihmStatus.nb_states = 3;

                outMsg.payload.ihmStatus.states[0].id = IHM_PRESENCE_1;
                outMsg.payload.ihmStatus.states[0].state.state_presence = eIhmPresence(presence1);

                outMsg.payload.ihmStatus.states[1].id = IHM_PRESENCE_2;
                outMsg.payload.ihmStatus.states[1].state.state_presence = eIhmPresence(presence2);

                outMsg.payload.ihmStatus.states[2].id = IHM_PRESENCE_3;
                outMsg.payload.ihmStatus.states[2].state.state_presence = eIhmPresence(presence3);

                while( (ret = bn_send(&outMsg)) <= 0);

                outMsg.header.destAddr = inMsg.header.srcAddr;
                outMsg.header.type = E_IHM_STATUS;
                outMsg.header.size = 2 + 3*sizeof(*outMsg.payload.ihmStatus.states);

                outMsg.payload.ihmStatus.nb_states = 2;

                outMsg.payload.ihmStatus.states[0].id = IHM_PRESENCE_4;
                outMsg.payload.ihmStatus.states[0].state.state_presence = eIhmPresence(presence4);

                outMsg.payload.ihmStatus.states[1].id = IHM_PRESENCE_5;
                outMsg.payload.ihmStatus.states[1].state.state_presence = eIhmPresence(presence5);

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

        	while( (ret = bn_sendAck(&outMsg)) <= 0);
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
        	while((ret = bn_send(&outMsg)) <=0);

        }

        flagModeSwitch = 0;
    }


    	if (!flagPresence1){
    		presence1Old = presence1;
    		presence1 = digitalRead(PIN_PRESENCE_1);

    		if (presence1Old != presence1){
    			flagPresence1 = 1;
    			timePresence1 = time;
    			presence1Old = presence1;
    		}
    	}
        if (flagPresence1 && (time-timePresence1 >= 40)){
        	presence1 = digitalRead(PIN_PRESENCE_1);
        	if (presence1Old == presence1){
        		outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        		outMsg.header.type = E_IHM_STATUS;
        		outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        		outMsg.payload.ihmStatus.nb_states = 1;
        		outMsg.payload.ihmStatus.states[0].id = IHM_PRESENCE_1;
        		outMsg.payload.ihmStatus.states[0].state.state_presence = eIhmPresence(presence1);
        		while( (ret = bn_send(&outMsg)) <= 0);
#ifdef DEBUG
        		Serial.println(ret);
#endif
        	}
        	flagPresence1 = 0;
        }




    if (!flagPresence2){
		presence2Old = presence2;
   		presence2 = digitalRead(PIN_PRESENCE_2);

   		if (presence2Old != presence2){
    		flagPresence2 = 1;
    		timePresence2 = time;
    		presence2Old = presence2;

    	}
    }

    if (flagPresence2 && time-timePresence2 >= 40){
    	presence2 = digitalRead(PIN_PRESENCE_2);
       	if (presence2Old == presence2){
       		outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
    		outMsg.header.type = E_IHM_STATUS;
    	    outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
    	    outMsg.payload.ihmStatus.nb_states = 1;
    	    outMsg.payload.ihmStatus.states[0].id = IHM_PRESENCE_2;
    	    outMsg.payload.ihmStatus.states[0].state.state_presence = eIhmPresence(presence2);
    	    while( (ret = bn_send(&outMsg)) <= 0);
       	}
       	flagPresence2 = 0;
    }



    if (!flagPresence3){
    	presence3Old = presence3;
    	presence3 = digitalRead(PIN_PRESENCE_3);

    	if (presence3Old != presence3){
    			flagPresence3 = 1;
    			timePresence3 = time;
    			presence3Old = presence3;
    		}
    	}
        if (flagPresence3 && time-timePresence3 >= 40){
        	presence3 = digitalRead(PIN_PRESENCE_3);
        	if (presence3Old == presence3){
        		outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        			outMsg.header.type = E_IHM_STATUS;
        		    outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        		    outMsg.payload.ihmStatus.nb_states = 1;
        		    outMsg.payload.ihmStatus.states[0].id = IHM_PRESENCE_3;
        		    outMsg.payload.ihmStatus.states[0].state.state_presence = eIhmPresence(presence3);
        		    while( (ret = bn_send(&outMsg)) <= 0);
        	}
        	flagPresence3 = 0;
        }




    	if (!flagPresence4){
    		presence4Old = presence4;
    		presence4 = digitalRead(PIN_PRESENCE_4);

    		if (presence4Old != presence4){
    			flagPresence4 = 1;
    			timePresence4 = time;
    			presence4Old = presence4;
    		}
    	}
        if (flagPresence4 && time-timePresence4 >= 40){
        	presence4 = digitalRead(PIN_PRESENCE_4);
        	if (presence4Old == presence4){

        		outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        			outMsg.header.type = E_IHM_STATUS;
        		    outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        		    outMsg.payload.ihmStatus.nb_states = 1;
        		    outMsg.payload.ihmStatus.states[0].id = IHM_PRESENCE_4;
        		    outMsg.payload.ihmStatus.states[0].state.state_presence = eIhmPresence(presence4);
        		    while( (ret = bn_send(&outMsg)) <= 0);
        	}
        	flagPresence4 = 0;
        }



    	if (!flagPresence5){
    		presence5Old = presence5;
    		presence5 = digitalRead(PIN_PRESENCE_5);

    		if (presence5Old != presence5){
    			flagPresence5 = 1;
    			timePresence5 = time;
    			presence5Old = presence5;
    		}
    	}
        if (flagPresence5 && time-timePresence5 >= 40){
        	presence5 = digitalRead(PIN_PRESENCE_5);
        	if (presence5Old == presence5){
        		outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        			outMsg.header.type = E_IHM_STATUS;
        		    outMsg.header.size = 2 + 1*sizeof(*outMsg.payload.ihmStatus.states);
        		    outMsg.payload.ihmStatus.nb_states = 1;
        		    outMsg.payload.ihmStatus.states[0].id = IHM_PRESENCE_5;
        		    outMsg.payload.ihmStatus.states[0].state.state_presence = eIhmPresence(presence5);
        		    while( (ret = bn_send(&outMsg)) <= 0);
        	}
        	flagPresence5 = 0;
        }

}

void fctModeSwitch(void){
    timeModeSwitch = time;
    debounceModeSwitch = digitalRead(PIN_MODE_SWITCH);
    flagModeSwitch = 1;
}

void fctStartingCord(void){
    timeStartingCord = time;
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
