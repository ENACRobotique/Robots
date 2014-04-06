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
#include "lib_sync_turret.h"
#include "roles.h"
#include "loc_tools_turret.h"
#include "../../../global_errors.h"

mainState state=S_SYNC_ELECTION;

sDeviceInfo devicesInfo[D_AMOUNT];
int iDevice;
int lastIndex=0;    // to detect new turn in game state

sMsg inMsg,outMsg;

unsigned long sw=0, sw2=0;

void setup(){

    setupFreeTest();

    // initializes the local state of the beacons to "off"
    for (int i=0;i<D_AMOUNT;i++) {
        devicesInfo[i].state=DS_OFF;
        devicesInfo[i].lastIndex=-1;
    }
    devicesInfo[D_MOBILE_1].addr=ADDRX_MOBILE_1;
    //fixme : do the same for the others
    domi_init(2);

    bn_init();

    bn_attach(E_ROLE_SETUP,role_setup);
    bn_printfDbg("start turret, free mem : %d o\n",freeMemory());

    pinMode(PIN_DBG_LED,OUTPUT);
}


void loop(){

    int rxB=0; //received bytes (size of inMsg when a message has been received)
    static unsigned long time_prev_period=0;


#ifdef BLINK_1S
    static unsigned long time_prev_led=0;
    static char debug_led=0;
#endif

    unsigned long time=millis();

///////// must always be done, any state

    //eventual receiving && routine
    rxB=bn_receive(&inMsg);  // rxB>0 if message written in inMsg, <0 if error

    //period broadcast : one by one TODO : broadcast
    if((time - time_prev_period)>=ROT_PERIOD_BCAST) {
        time_prev_period = millis();
        outMsg.header.destAddr=ADDRX_MOBILE_1; //FIXME : to everybody
        outMsg.header.type=E_PERIOD;
        outMsg.header.size=sizeof(outMsg.payload.period);
        outMsg.payload.period=domi_meanPeriod();
        bn_send(&outMsg);
    }
    //blinking
#ifdef BLINK_1S
    if((time - time_prev_led)>=10000) {
        time_prev_led = millis();
        digitalWrite(PIN_DBG_LED,debug_led^=1);
//        bn_printfDbg((char*)"turret %lu, mem : %d, state : %d\n",millis()/1000,freeMemory(),state);
    }
#endif

    ///////state machine
    switch (state){

    case S_SYNC_ELECTION :
        if (!sw){
            //tell beacon(s) to begin election
            for (int i=0;i<D_AMOUNT;i++){
                if (sync_beginElection(devicesInfo[i].addr)>0) devicesInfo[i].state=DS_UNSYNCED;
            }

            sw=micros();
        }
        //wait 2s
        else if (micros()-sw>ELECTION_TIME){
            //switch to next state (beacon will switch automatically on reception of the first data packet)
            state=S_SYNC_MEASURE;
            bn_printDbg("end election\n");
            sw=micros();
        }
        break;  //

    case S_SYNC_MEASURE:
        //during the defined synchronization time
        if (micros()-sw<SYNCRONIZATION_TIME){
            // $iStates (device to sync) send data if a new turn has been detected since the last time we have send data to this particular device
            if (domi_nbTR()!=devicesInfo[iDevice].lastIndex){
                devicesInfo[iDevice].lastIndex=domi_nbTR();

                //send data
                sync_sendData(devicesInfo[iDevice].addr);
                //increase index for device selection
                iDevice=(iDevice+1)%D_AMOUNT;
            }
        }
        else {
            // at the end, change state
            state=S_SYNC_END;
            bn_printDbg("end data collection\n");
        }
        break;

    case S_SYNC_END :
        //send the SYNCF_END_MESURES
        for (int i=0;i<D_AMOUNT;i++){
            if (sync_sendEnd(devicesInfo[i].addr)>0) devicesInfo[i].state=DS_GAME;
        }
        state=S_GAME;
        break;

    case S_GAME :
        // if new turn
        if (lastIndex!=domi_nbTR()){
            lastIndex=domi_nbTR();
            // handle every payload eventually waiting
            for (int i=0;i<D_AMOUNT;i++){
                if (devicesInfo[i].lastData.value){
                    if ( handleMeasurePayload(&(devicesInfo[i].lastData),devicesInfo[i].addr)!=-ERR_TRY_AGAIN ) {
                        memset(&devicesInfo[i].lastData,'0',sizeof(sMobileReportPayload));
                    }
                }
            }
        }

        //if message received
        if (rxB>0){
            switch (inMsg.header.type){
            // if measure message received
            case E_MEASURE :
                int error;
                switch (inMsg.header.srcAddr){
                case ADDRX_MOBILE_1 :
                    if ( (error=handleMeasurePayload(&(inMsg.payload.mobileReport),inMsg.header.srcAddr))==-ERR_TRY_AGAIN ) {
                        memcpy(&devicesInfo[D_MOBILE_1].lastData,&inMsg.payload.mobileReport,sizeof(sMobileReportPayload));
                    }
                    break;
                case ADDRX_MOBILE_2 :
                    break;
                case ADDRX_SECOND :
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
    }// end state machine
}
