/*
 * balise_principal.cpp
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#include "Arduino.h"
#include "inttypes.h"
#include "float.h"
#include "MemoryFree.h"

#include "messages.h"
#include "network_cfg.h"
#include "params.h"
#include "../../../communication/botNet/shared/botNet_core.h"
#include "lib_domitille.h"
#include "../../../communication/botNet/shared/bn_debug.h"

uint32_t mesTab[2]={0,0};

void setup(){
    bn_init();

    bn_attach(E_DEBUG_SIGNALLING,&bn_debugUpdateAddr);
    bn_printfDbg("start main turret beacon, free mem : %d o\n",freeMemory());

//    domi_init(2);

    pinMode(PIN_DBG_LED,OUTPUT);
}

int state=SYNC;
#ifdef BLINK_1S
int debug_led=0;
#endif
int period2=0;
char syncOKbool=0;
sMsg inMsg,outMsg;
sMesPayload last1,last2,lastS; //last measure send by mobile 1,  2, secondary

int routineErr=0,i=0,msg2send=100,msgNOk=0,msgNStatused=0,msgSend=0;
unsigned long avgMes=0,prevMsg=0,avgElem=0,avgVal=0;
sMsg out,in;
unsigned long sw=0;
int led=1;

void loop(){

#if 0
    if (bn_routine()<0) {
        routineErr++;
    }
    if (bn_receive(&in)){
        i++;
    }

    out.header.destAddr=ADDRX_MOBILE_1;
    out.header.size=40;
    out.header.type=E_PERIOD;
    if ( msg2send > 0 ){
        avgMes=(micros()-prevMsg);
        avgElem++;
        avgVal= avgMes + avgVal*(avgElem-1);
        avgVal/= avgElem;
        prevMsg=micros();
        switch (bn_send(&out)){
        case -3 : msgNOk++; break;
        case -2 : msgNStatused++; break;
        case -1 : msgNStatused++; break;
        default : msgSend++; break;
        }
        msg2send--;
    }

    if (msg2send==0){
        bn_printfDbg("send %d, Nstat %d, NOk %d, avg int %lu \n",msgSend, msgNStatused,msgNOk,avgVal);
        msg2send--;
    }

    if ( millis()-sw > 1000){
        sw=millis();
        led^=1;
        digitalWrite(PIN_DBG_LED,led);
        bn_printfDbg("blink, %lu s, free mem : %d, rx : %d, routiEr %d\n",millis()/1000,freeMemory(),i,routineErr);
    }
#elif 1
//    int rxB=0; //received bytes (size of inMsg when a message has been received)
//    static unsigned long time_prev_period=0,prev_TR=0;
#ifdef BLINK_1S
    static unsigned long time_prev_led=0;
#endif

    unsigned long time=millis();

///////// must always be done, any state

    //eventual receiving && routine
    /*rxB=*/bn_receive(&inMsg);

    time=millis();

    //blinking
#ifdef BLINK_1S
    if((time - time_prev_led)>=3000) {
        time_prev_led = millis();
        digitalWrite(PIN_DBG_LED,debug_led^=1);
        bn_printfDbg("main turret beacon %lu, mem : %d, state : %d\n",millis()/1000,freeMemory(),state);
    }
#endif
#if 0
    //period broadcast : one by one
    if((time - time_prev_period)>=ROT_PERIOD_BCAST) {
        out.header.destAddr=ADDRX_MOBILE_1;
        out.header.size=4;
        out.header.type=E_PERIOD;
//        bn_send(&out);

        time_prev_period = millis();
        period2++;
        period2=(period2)%5;
//        switch (period2){
//            case 0 :
                outMsg.header.destAddr=ADDRX_MOBILE_1;
//                break;
//            case 1 : outMsg.header.destAddr=ADDRX_MOBILE_2; break;
//            case 2 : outMsg.header.destAddr=ADDRX_SECOND; break;
//            case 3 : outMsg.header.destAddr=ADDRX_FIX; break;
//            case 4 : outMsg.header.destAddr=debug_addr; break;
//            default : break;
//        }
        outMsg.header.type=E_PERIOD;
        outMsg.header.size=sizeof(outMsg.payload.period);
        outMsg.payload.period=domi_meanPeriod();
        int sndVal=bn_send(&out);
        //bn_printfDbg("period send to %hx %lu, err : %d\n",outMsg.header.destAddr,outMsg.payload.period,sndVal);
    }
#endif
#else
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
                bn_send(&outMsg);
                state=GAME;
            }
            rxB=0;
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
                          bn_send(&outMsg);
                      }
//FIXME                      if (!(syncOKbool & (ADDRX_MOBILE_2&DEVICE_MASK))){
//                          //expected for balise 2
//                          outMsg.header.destAddr=ADDRX_MOBILE_2;
//                          outMsg.header.type=E_SYNC_EXPECTED_TIME;
//                          outMsg.header.size=sizeof(outMsg.payload.syncTime);
//                          outMsg.payload.syncTime=((TR_period*PHASE_INIT_MOBILE_2)>>9)+prev_TR;
//                          bn_send(&outMsg);
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
#endif
}
