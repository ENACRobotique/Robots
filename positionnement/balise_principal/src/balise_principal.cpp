/*
 * balise_principal.cpp
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#include "lib_domitille.h"

int freeRam () {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

int usedDataRam () {
    extern int __heap_start, __data_start;
    return (int) &__heap_start - (int) &__data_start;
}

#include "Arduino.h"
#include "Wire.h"
#include "inttypes.h"

#include "messages.h"
#include "network_cfg.h"
#include "params.h"
#include "lib_superBus.h"
#include "lib_checksum.h"

#include "lib_I2C_arduino.h"

uint32_t mesTab[2]={0,0};

void setup(){
    sb_init();

    pinMode(PIN_DBG_LED,OUTPUT);
    pinMode(PIN_RST_XBEE,OUTPUT);
    //setupXbee();
    digitalWrite(PIN_RST_XBEE,HIGH);
    delay(100);
    digitalWrite(PIN_RST_XBEE,LOW);
    delay(200);

    while(Serial.read() != -1); //flush the buffer for any incorrect bytes

    domi_init(2);

#ifdef DEBUG
    sb_printDbg(ADDRX_DEBUG,"start turret",freeRam(),usedDataRam());
#endif
}

static int state=GAME,debug_led=0;
char syncOKbool=0;
sMsg inMsg,outMsg;
sMesPayload last1,last2,lastS; //last measure send by mobile 1,  2, secondary

void loop(){
    int rxB=0; //received bytes (size of inMsg when a message has been received)
    int nbRoutine=0; //number of call to sb_routine during this loop
    static unsigned long time_prev_led=0,time_prev_period=0,prev_TR=0;

    unsigned long time=millis();

///////// must always be done, any state

    //network routine
    while (sb_routine() && nbRoutine<MAX_ROUTINE_CALL && (millis()-time)<MAX_ROUTINE_TIME) {
        nbRoutine++;
    }
    //eventual receiving
    rxB=sb_receive(&inMsg);

    time=millis();

    //blinking
#ifdef BLINK_1S
    if((time - time_prev_led)>=500) {
        time_prev_led = millis();
        digitalWrite(PIN_DBG_LED,debug_led^=1);
    }
#endif
    //period broadcast
    if((time - time_prev_period)>=ROT_PERIOD_BCAST) {
        time_prev_period = millis();

        outMsg.header.destAddr=ADDRX_BROADCAST;
        outMsg.header.type=E_PERIOD;
        outMsg.header.size=sizeof(outMsg.payload.period);
        outMsg.payload.period=domi_meanPeriod();
        sb_send(&outMsg);
    }

    //some message handling
    if (rxB){
        switch (inMsg.header.type){
        case E_SYNC_OK :
            //write in syncOkbool that the sender is in sync
            syncOKbool|=(inMsg.header.srcAddr&DEVICEX_MASK);
            //if everybody is in sync, send a global "SYNC_OK"
            if (syncOKbool==(ADDRX_MOBILE_1)){//FIXME | ADDRX_MOBILE_2 | ADDRX_SECOND
                outMsg.header.destAddr=ADDRX_BROADCAST;
                outMsg.header.type=E_SYNC_OK;
                outMsg.header.size=0;
                sb_send(&outMsg);
                state=GAME;
            }
            break;
        default : break;
        }
    }

///////state machine
      switch (state){
          case SYNC:
                  //if nouveau tour, envoyer les expected pour le dernier tour
                  if(prev_TR!=last_TR){
                      //send to the laser receiver the expected time they should have seen the laser in order to synchronize the clocks
                      if (!(syncOKbool & (ADDRX_MOBILE_1&DEVICEX_MASK))){
                          //expected for balise 1
                          outMsg.header.destAddr=ADDRX_MOBILE_1;
                          outMsg.header.type=E_SYNC_EXPECTED_TIME;
                          outMsg.header.size=sizeof(outMsg.payload.syncTime);
                          outMsg.payload.syncTime=((TR_period*PHASE_INIT_MOBILE_1)>>9)+prev_TR;
                          sb_send(&outMsg);
                      }
//FIXME                      if (!(syncOKbool & (ADDRX_MOBILE_2&DEVICE_MASK))){
//                          //expected for balise 2
//                          outMsg.header.destAddr=ADDRX_MOBILE_2;
//                          outMsg.header.type=E_SYNC_EXPECTED_TIME;
//                          outMsg.header.size=sizeof(outMsg.payload.syncTime);
//                          outMsg.payload.syncTime=((TR_period*PHASE_INIT_MOBILE_2)>>9)+prev_TR;
//                          sb_send(&outMsg);
//                      }
                      prev_TR=last_TR;
                  }
                  break;
          case GAME :
                  if (rxB){
                      switch (inMsg.header.type){
                      case E_MEASURE :
                          switch (inMsg.header.srcAddr){
                          case ADDRX_MOBILE_1 :
                              last1=inMsg.payload.measure;
                              break;
                          case ADDRX_MOBILE_2 :
                              last2=inMsg.payload.measure;
                              break;
                          case ADDRX_SECOND :
                            lastS=inMsg.payload.measure;
                            break;
                          default : break;
                          }
                          rxB=0;
                          break;

                      default : break;
                      }
                  }
                  break;
          default : break;
      }
}

