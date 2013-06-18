/*
 * main.c
 *
 *  Created on: 17 juin 2013
 *      Author: quentin
 */

#include "Arduino.h"
#include "Xbee_API.h"
#include "params.h"
#include <stdio.h>
#include <string.h>



spAPISpecificStruct struOut={0},struIn={0};
int diffAck=0, diffStat=0;
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

#include <stdarg.h>
void spf(char * s,char *fmt, ... ){
        va_list args;
        va_start (args, fmt );
        vsnprintf(s, 32, fmt, args);
        va_end (args);
}


void setup(){

    pinMode(13,OUTPUT);
    digitalWrite(13,LOW);


    pinMode(PIN_RST_XBEE,OUTPUT);

    serialInit(111111,0);
    delay(100);

    //hard reset
    digitalWrite(PIN_RST_XBEE,HIGH);
    delay(10);
    digitalWrite(PIN_RST_XBEE,LOW);
    delay(500);

    // forget about -> /!\ for some unknown reason, MUST BE DONE (arduino Serial overflow heavily suspected)
    // problem solved itself. maybe it will reappear
    //while(!XbeeReadFrame(&struIn)) blink(5);
    //blink(2);


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

    blink(4);
    XbeeTx16(0x5678,XBEE_TX_O_NOACK,0,"hello world!",13);
}

void loop(){
    char string[32]={0};
    int recBytes=0;
    //receive Frame
    recBytes=XbeeReadFrame(&struIn);

    //if msg received respond pong
    if(recBytes){
        if (struIn.APID==XBEE_APID_RX16){
            delay(10);
            spf(string,"pong n%d a%d s%d",struIn.data.RX16Data.payload[0],diffAck,diffStat);
            XbeeTx16(0x5678,0,0x55,string,strlen(string)+1);
            diffAck++;
            diffStat++;
        }
        //if ack, decrement
        else if (struIn.APID == XBEE_APID_TXS){
            if (struIn.data.TXStatus.status == XBEE_TX_S_SUCCESS){
                diffAck--;
            }
            diffStat--;
        }

    }

    if ((millis()-prevLed) > 500){
        prevLed=millis();
        led^=1;
        digitalWrite(13,led);
    }

}
