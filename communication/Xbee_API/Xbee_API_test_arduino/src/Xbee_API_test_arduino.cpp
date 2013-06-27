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

unsigned long int send=0;
unsigned long int statused=0, acked=0;
int badCS=0, timeOut=0;
unsigned long  totalMicros=0;
unsigned long prevTime, stopWatch, mean=0, mes=0;



#define NB_TEST 10000
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
        vsnprintf(s, 64, fmt, args);
        va_end (args);
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

    //blink(4);
    //XbeeTx16(0x5678,XBEE_TX_O_NOACK,0,"hello world!",13);

    Wire.beginTransmission(0x02);
    Wire.println("end init");
    Wire.endTransmission();
#ifdef SIMPLEPING
    //send frame
    XbeeTx16(0x9999,0,42,"ping",5);
    stopWatch=micros();
    send++;
#endif
}



void loop(){
    char string[64]={0};
    int recBytes=0;

#ifdef SIMPLEPING  //simple ping sender
    if (send<NB_TEST){

        //read status
        recBytes=XbeeReadFrame(&struIn);

        if (recBytes > 0){
            if (struIn.APID==XBEE_APID_TXS){
                statused++;
                if (struIn.data.TXStatus.status==XBEE_TX_S_SUCCESS) {
                    acked++;
                }
                else {
                    Wire.beginTransmission(0x02);
                    Wire.println("not acked !");
                    Wire.endTransmission();
                }
            }
            else if (struIn.APID==XBEE_APID_RX16){
                mes=micros()-stopWatch;
                mean=(mean*(send-1)+mes)/send;

                //print dbg
                Wire.beginTransmission(0x02);
                sprintf(string,"%lu send, ping : %lu | %lu",send,mes,mean);
                Wire.write((uint8_t *)string,strlen(string));
                Wire.println("");
                Wire.println(mes);
                Wire.endTransmission();


                //start timer & send frame
                stopWatch=micros();
                XbeeTx16(0x9999,0,0,"ping",35);
                send++;
            }
        }
        else if (recBytes==-1){
            badCS++;
        }

        memset(&struIn,0,sizeof(&struIn));


    }
    if (send==NB_TEST){

        Wire.beginTransmission(0x02);
        spf(string,"total for %d : ",send);
        Wire.write((uint8_t *)string,strlen(string));
        Wire.println((millis()-prevTime)*1000);
        Wire.endTransmission();

        Wire.beginTransmission(0x02);
        spf(string,"send : %ld stat : %ld ack : %ld !CS : %d TO : %d",send,statused,acked,badCS,timeOut);
        Wire.write((uint8_t *)string,strlen(string));
        Wire.println("");
        Wire.endTransmission();

        send++;
    }
#endif

#ifdef BURST   //burst ping sender

    for (send=0;send<10;send++){
        XbeeTx16(0x9999,0,0,"ping",5);
    }
    //read status
    while (!(recBytes=XbeeReadFrame(&struIn)));

    if (recBytes > 0){
        if (struIn.APID==XBEE_APID_RX16){
            //print dbg
            Wire.beginTransmission(0x02);
            spf(string,"%d send, received : %d",send,struIn.data.RX16Data.payload);
            Wire.write((uint8_t *)string,strlen(string));
            Wire.println("");
            Wire.endTransmission();
        }
        else {
            Wire.beginTransmission(0x02);
            Wire.println("???");
            Wire.endTransmission();
        }
    }
    else if (recBytes==-1){
        //print dbg
       Wire.beginTransmission(0x02);
       Wire.println("Wrong Xbee frame CS");
       Wire.endTransmission();
    }

    memset(&struIn,0,sizeof(&struIn));


#endif
    if ((millis()-prevLed) > 500){
        prevLed=millis();
        led^=1;
        digitalWrite(13,led);
    }

}
