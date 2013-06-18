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



spAPISpecificStruct struOut,struIn;
int send=0, acked=0, statused=0, diff=0;
unsigned long prevLed=0;
int led=0;

void setup(){

    pinMode(13,OUTPUT);
    digitalWrite(13,LOW);

    serialInit(111111,0);
    XbeeATCmd("MY", 42, XBEE_ATCMD_SET, MYADDRI);

    //waits for command acknowledgement
    do {
        XbeeReadFrame(&struIn);
    }while (struIn.APID!=XBEE_APID_ATRESPONSE || struIn.data.ATResponse.frameID!=42);

    //writes config
    XbeeATCmd("WR", 0, XBEE_ATCMD_SET, MYADDRI);

    XbeeTx16(0x5678,XBEE_TX_O_NOACK,0,"hello world",12);
}

void loop(){
    char string[32];
    int recBytes;
    //receive Frame
    recBytes=XbeeReadFrame(&struIn);

    //if msg received respond pong
    if(recBytes){
        if (struIn.APID==XBEE_APID_RX16){
            sprintf(string,"pong %d %d",diff,send-statused);
            XbeeTx16(hbe2_swap(struIn.data.RX16Data.lSrcAddr_be),0,0x55,string,strlen(string));
            send++;
            diff++;
        }
        //if ack, decrement
        else if (struIn.APID == XBEE_APID_TXS){
            if (struIn.data.TXStatus.status == XBEE_TX_S_SUCCESS){
                acked++;
                diff--;
            }
            statused++;
        }

        led^=1;
        digitalWrite(13,led);
    }

    if ((millis()-prevLed) > 500){
        prevLed=millis();

    }

}
