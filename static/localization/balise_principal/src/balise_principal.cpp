/*
 * balise_principal.cpp
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */



#include "Arduino.h"
#include "Wire.h"
#include "inttypes.h"
#include "float.h"
#include "MemoryFree.h"

#include "messages.h"
#include "network_cfg.h"
#include "params.h"
#include "lib_superBus.h"
#include "lib_checksum.h"
#include "lib_domitille.h"
#include "lib_sbDebug.h"

uint32_t mesTab[2]={0,0};


void setup(){

    sb_init();

    sb_attach(E_DEBUG_SIGNALLING,&sb_debugUpdateAddr);
    sb_printfDbg("start balise 1, free mem : %d o\n",freeMemory());

    domi_init(2);


}

int state=SYNC;
#ifdef BLINK_1S
int debug_led=0;
#endif
int period2=0;
char syncOKbool=0;
sMsg inMsg,outMsg;
sMesPayload last1,last2,lastS; //last measure send by mobile 1,  2, secondary

void loop(){
    int rxB=0; //received bytes (size of inMsg when a message has been received)
    static unsigned long time_prev_period=0,prev_TR=0;
#ifdef BLINK_1S
    static unsigned long time_prev_led=0;
#endif

    unsigned long time=millis();

///////// must always be done, any state

    //network routine
    sb_routine();

    //eventual receiving
    rxB=sb_receive(&inMsg);

    time=millis();

    //blinking
#ifdef BLINK_1S
    if((time - time_prev_led)>=3000) {
        time_prev_led = millis();
        digitalWrite(PIN_DBG_LED,debug_led^=1);
        sb_printfDbg("turret %lu, mem : %d, state : %d\n",millis()/1000,freeMemory(),state);
    }
#endif
    //period broadcast : one by one
    if((time - time_prev_period)>=ROT_PERIOD_BCAST) {
        time_prev_period = millis();
        period2++;
        period2%=5;
        switch (period2){
        case 0 : outMsg.header.destAddr=ADDRX_MOBILE_1; break;
        case 1 : outMsg.header.destAddr=ADDRX_MOBILE_2; break;
        case 2 : outMsg.header.destAddr=ADDRX_SECOND; break;
        case 3 : outMsg.header.destAddr=ADDRX_FIX; break;
        case 4 : outMsg.header.destAddr=ADDR_DEBUG_DFLT; break;
        default : break;
        }
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

