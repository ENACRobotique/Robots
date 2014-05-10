/*
 * main.cpp
 *
 *  Created on: 19 janv. 2014
 *      Author: quentin
 */


#define PIN_DBG_LED 13
#define PIN_RST_XBEE 5


#include "Arduino.h"

#include "messages.h"
#include "../../../botNet/shared/botNet_core.h"
#include "../../../network_tools/bn_debug.h"
#include "../../../botNet/shared/Xbee4bn.h"
#include "network_cfg.h"
#include "MemoryFree.h"
#include "../../testNet_functions/bn_testFunc.h"

unsigned long int sw; //stopwatch for Xbee startup delay
int led=1;
int msg2send=0,msgSend=0,msgNStatused=0,msgNOk=0,routineErr=0;
unsigned long prevMsg=0;
int avgElem=0;
unsigned long avgMes=0, avgVal=0;

sMsg in,out;

void setup() {
    sw=millis();
    bn_init();


    bn_attach(E_ROLE_SETUP,role_setup);
    bn_printfDbg("start beacon 2, free mem : %d o\n",freeMemory());


    pinMode(13,OUTPUT);
    prevMsg=micros();


}

void loop() {
    if (bn_routine()<0) {
        routineErr++;
    }
    if (bn_receive(&in)){

    }




    if ( millis()-sw > 1000){
        sw=millis();
        led^=1;
        digitalWrite(13,led);
        bn_printfDbg("%lu s, free mem : %d, routiEr %d\n",millis()/1000,freeMemory(),routineErr);
    }

}

