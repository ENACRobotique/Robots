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
#include "../../../communication/network_tools/bn_debug.h"
#include "loc_tools_turret.h"
#include "global_errors.h"
#include "messages.h"
#include "bn_utils.h"
extern "C" {
#include "roles.h"
#include "global_sync.h"
}

#ifdef SYNC_WIRELESS
#include "shared/lib_synchro_laser_turret.h"
#endif

#ifdef SYNC_WIRED
#include "shared/lib_synchro_wire.h"

#endif




mainState state=S_CHECKREMOTE;


sDeviceInfo devicesInfo[D_AMOUNT];
int iDeviceSync=0,iDevicePeriodBcast=0;
int lastIndex=0;    // to detect new turn in game state
uint32_t endSync = 0;
sMsg inMsg,outMsg;

unsigned long sw=0, sw2=0;

void setup(){

    setupFreeTest();

#ifdef SYNC_WIRED
    wiredSync_senderInit(PIN_SYNC);
#endif
    // initializes the local state of the beacons to "off"
    for (int i=0;i<D_AMOUNT;i++) {
        devicesInfo[i].state=DS_OFF;
        devicesInfo[i].lastIndex=-1;
    }
    devicesInfo[D_MOBILE_1].addr=ADDRX_MOBILE_1;
    devicesInfo[D_MOBILE_2].addr=ADDRX_MOBILE_2;
//    devicesInfo[D_FIX].addr=ADDRX_FIX;

    //fixme : do the same for the others

    domi_init(2,9);

    bn_init();

    bn_attach(E_ROLE_SETUP,role_setup);
    bn_attach(E_SYNC_QUERY,gs_receiveQuery);
#ifdef DEBUG
    bn_printfDbg("start turret, free mem : %d o\n",freeMemory());
#endif

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
    if (rxB>0) {
        switch (inMsg.header.type) {
        case E_SYNC_STATUS :
            for (int i=0; i<D_AMOUNT; i++){
                if (devicesInfo[i].addr == inMsg.header.srcAddr) {
                    if (inMsg.payload.syncWired.flag == SYNC_OK){
                        devicesInfo[i].state = DS_SYNCED;
                    }
                }
            }
            break;
        default :
            break;
        }
    }

    //period broadcast : one by one TODO : broadcast
    if((time - time_prev_period)>=ROT_PERIOD_BCAST) {
        time_prev_period = millis();

        //bcast to current device
        outMsg.header.destAddr=devicesInfo[iDevicePeriodBcast].addr;
        outMsg.header.type=E_PERIOD;
        outMsg.header.size=sizeof(outMsg.payload.period);
        outMsg.payload.period=domi_meanPeriod();
        bn_send(&outMsg);

        //increment index for next device
        iDevicePeriodBcast=(iDevicePeriodBcast+1)%D_AMOUNT;
    }

    //blinking
#ifdef BLINK_1S
    if((time - time_prev_led)>=1000) {
        time_prev_led = millis();
        digitalWrite(PIN_DBG_LED,debug_led^=1);
#ifdef DEBUG_SYNC_WIRE_EVAL
        if (millis() > 35000){
            wiredSync_setSignal(WIREDSYNC_SIGNALISHERE);
            delay(WIREDSYNC_LOWTIME/1000);
            wiredSync_setSignal(WIREDSYNC_SIGNANOTHERE);
            uint32_t end = micros();
            bn_printfDbg("tur, %lu,",micros2sl(end));
        }
#endif
#ifdef DEBUG
    bn_printfDbg("%lu period %lu %lu",micros(),domi_meanPeriod(),domi_lastTR());
//        bn_printfDbg((char*)"turret %lu, mem : %d, state : %d\n",millis()/1000,freeMemory(),state);
#endif
#if defined(DEBUG_CALIBRATION) && defined(DEBUG_CALIBRATION_speed)
    static int setSpeed=0;
    switch (setSpeed){
    case 0 : setSpeed=1;
        domi_setspeed(SPEED_HIGH);
        break;
    case 1 : setSpeed=2;
        domi_setspeed(SPEED_20HZ);
        break;
    case 2 : setSpeed=0;
        domi_setspeed(SPEED_SLOW);
        break;
    }
#endif
    }
#endif

    if (gs_isBeaconRequested() && (state != S_CHECKREMOTE
                                    || state != S_SYNC_MEASURE
                                    || state != S_SYNC_ELECTION
                                    || state != S_SYNC_END) ){
        state = S_CHECKREMOTE;

    }

    ///////state machine
    switch (state){
    case S_CHECKREMOTE :
        int switchState ;
        switchState = 1;
        // check if every device is on
        for (int i=0;i<D_AMOUNT;i++) {
            if (int err=bn_ping(devicesInfo[i].addr)<0){
                switchState = 0;
                if (gs_isBeaconRequested()) {
                    outMsg.header.destAddr = gs_getBeaconQueryOrigin();
                    outMsg.header.type = E_SYNC_RESPONSE;
                    outMsg.header.size = sizeof(outMsg.payload.syncResponse.cfgs[0]);
                    outMsg.payload.syncResponse.nb = 1;
                    outMsg.payload.syncResponse.cfgs[0].type = SYNCTYPE_ADDRESS;
                    outMsg.payload.syncResponse.cfgs[0].addr = devicesInfo[i].addr;
                    outMsg.payload.syncResponse.cfgs[0].status = SYNCSTATUS_PING_KO;
                    while (bn_sendAck(&outMsg)<0); // critical, so loop.
                }

#ifdef DEBUG_SYNC
                bn_printfDbg("%hx offline (error : %d)\n",devicesInfo[i].addr,err);
#endif
            }
        }
        if (switchState) {
#ifdef SYNC_WIRED
            state = S_SYNC_MEASURE;
#elif defined(SYNC_WIRELESS)
            state = S_SYNC_ELECTION;
#endif
        }
        break;
#ifdef SYNC_WIRED
    case S_SYNC_MEASURE :
        if (wiredSync_sendSignal(0) != -1) {
            endSync = micros();
        }
        else {
            // wait until we receive the sync statuses
            int synced;
            synced=0;
            for (int i=0;i<D_AMOUNT;i++) {
                if (devicesInfo[i].state == DS_SYNCED) synced++;
            }
            if (synced == D_AMOUNT) {
                state = S_GAME;
                gs_beaconStatus(SYNCSTATUS_OK);
#ifdef DEBUG_SYNC
                bn_printDbg("sync ok\n");
#endif
            }
            if (endSync && (micros() - endSync) > 4*WIREDSYNC_PERIOD){
#ifdef DEBUG_SYNC
                for (int i=0;i<D_AMOUNT;i++) {
                    if (devicesInfo[i].state != DS_SYNCED) bn_printfDbg("%hx not synchronized (status %d)\n",devicesInfo[i].addr,devicesInfo[i].state);
                }
#endif
                gs_beaconStatus(SYNCSTATUS_SYNC_KO);
            }
        }
        break;
#endif
#ifdef SYNC_WIRELESS
    case S_SYNC_ELECTION :
        if (!sw){
            // set speed to high
            domi_setspeed(SPEED_HIGH);
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
            // set speed
            domi_setspeed(SPEED_20HZ);
        }
        break;  //

    case S_SYNC_MEASURE:

        //during the defined synchronization time
        if (micros()-sw<SYNCRONIZATION_TIME){
            // $iStates (device to sync) send data if a new turn has been detected since the last time we have send data to this particular device
            if (domi_nbTR()!=devicesInfo[iDeviceSync].lastIndex){
                devicesInfo[iDeviceSync].lastIndex=domi_nbTR();

                //send data
                sync_sendData(devicesInfo[iDeviceSync].addr);
                //increase index for device selection
                iDeviceSync=(iDeviceSync+1)%D_AMOUNT;
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
#endif
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
                    if ( (error=handleMeasurePayload(&(inMsg.payload.mobileReport),inMsg.header.srcAddr))==-ERR_TRY_AGAIN ) {
                        memcpy(&devicesInfo[D_MOBILE_2].lastData,&inMsg.payload.mobileReport,sizeof(sMobileReportPayload));
                    }
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
