/*
 * main.c
 *
 *  Created on: 17 juin 2013
 *      Author: quentin
 */

#include "Arduino.h"
#include "Wire.h"

#include "Xbee_API.h"
#include "params.h"


#include <stdio.h>
#include <string.h>



spAPISpecificStruct struOut={0},struIn={0};
unsigned long prevLed=0;
int led=0;

#define PIN_RST_XBEE 5

void blink(int n){
    int i;
    i=n;
    while (i-->0){
        digitalWrite(13,1);
        delay(100);
        digitalWrite(13,0);
        delay(200);
    }
    delay(1000);
}


void setup(){

    pinMode(13,OUTPUT);
    digitalWrite(13,LOW);


    pinMode(PIN_RST_XBEE,OUTPUT);

    serialInit(111111,0);
    delay(100);

    Wire.begin(0x04);

    Wire.beginTransmission(0x02);
    Wire.println("start mobile 2");
    Wire.endTransmission();

    //hard reset
    digitalWrite(PIN_RST_XBEE,HIGH);
    delay(10);
    digitalWrite(PIN_RST_XBEE,LOW);
    delay(500);


    //Writes my address
    XbeeATCmd("MY", 42, XBEE_ATCMD_SET, MYADDRI);
    //waits for command acknowledgment
    do {
        XbeeReadFrame(&struIn);
    }while (struIn.APID!=XBEE_APID_ATRESPONSE || struIn.data.ATResponse.frameID!=42);

    //store config
    XbeeATCmd("WR", 12, XBEE_ATCMD_GET, 0);
    //waits for command acknowledgment
    do {
        XbeeReadFrame(&struIn);
    }while (struIn.APID!=XBEE_APID_ATRESPONSE || struIn.data.ATResponse.frameID!=12);

}



int burstCount=0;
unsigned long burstLast=0;

void loop(){
    char string[64]={0};
    int recBytes=0;


#ifdef SIMPLEPING   //simple ping reply
    if (XbeeReadFrame(&struIn)>0){
        if (struIn.APID==XBEE_APID_RX16){
            XbeeTx16(0x1234,0,0,"pong",35);
        }
    }
#endif

#ifdef BURST   //burst  ping reply

    if (XbeeReadFrame(&struIn)>0){
        if (struIn.APID==XBEE_APID_RX16){
            burstLast=millis();
            burstCount++;
            digitalWrite(13,1);
        }
    }
    //after the burst
    if ((millis()-burstLast)>2000){
        XbeeTx16(0x1234,0,0,&burstCount,sizeof(burstCount));
        burstCount=0;
    }
#endif
//    if ((millis()-prevLed) > 500){
//        prevLed=millis();
//        led^=1;
//        digitalWrite(13,led);
//    }

}
