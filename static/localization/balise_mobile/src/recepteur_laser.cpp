#define PIN_DBG_LED 13



#include "Arduino.h"

#include "lib_int_laser.h"
//#include "lib_time.h"
#include "messages.h"
#include "../../../communication/botNet/shared/botNet_core.h"
#include "params.h"
#include "network_cfg.h"
#include "MemoryFree.h"
#include "lib_synchro_beacon.h"
#include "../../../communication/botNet/shared/bn_debug.h"


unsigned long lastLaserDetectMillis=0,lastLaserDetectMicros=0;
unsigned long time_prev_led=0, sw=0;
uint32_t time_prev_laser=0;



plStruct laserStruct0={0},laserStruct1={0}; // Structure storing laser detecion infos
volatile uint32_t laser_period=50000;       // in µs, to be confirmed by the main robot
uint32_t sync_lastTurnDate_stored=0,sync_period_stored=0,sync_last_received=0;
uint32_t lasStrRec0=0,lasStrRec1=0;         // date at which we updated the laser structure
int nbLas0=0, nbLas1=0;                     // count of laser interruption detected on channel n
char chosenOne=8;                           // interruption chosen for synchronization


char debug_led=1;
char state=S_SYNC_ELECTION, prevState=S_BEGIN;                           // State machine state

inline void periodHandle(sMsg *msg){
    if (msg->header.type==E_PERIOD)  laser_period=msg->payload.period;
}





void setup() {
  laserIntInit(0);
  laserIntInit(1);

  pinMode(13,OUTPUT);

  bn_init();

  bn_printDbg("start mobile 1");
  bn_attach(E_ROLE_SETUP,role_setup);
  bn_attach(E_PERIOD,&periodHandle);
  setupFreeTest();
}

void loop() {

    sMsg inMsg={{0}},outMsg={{0}};
    plStruct laserStruct={0}; //structure containing info about the last laser detection (deleted if not processed within one loop)
    int rxB=0; // size (bytes) of message available to read
    unsigned long time = millis(),timeMicros=micros();


    updateSync();
    //blink
    if((time - time_prev_led)>=3000) {
      time_prev_led= time;
      digitalWrite(PIN_DBG_LED,debug_led^=1);
#ifdef DEBUG
      bn_printfDbg((char*)"%lu, mem : %d, unused %d\n",time/1000,freeMemory(),getFreeTest());
#endif
    }

//MUST ALWAYS BE DONE (any state)


    // routine and receive
    rxB=bn_receive(&inMsg);

    //reading the eventual data from the lasers
    if (periodicLaser(&buf0,&laserStruct0)){
        lasStrRec0=timeMicros;
        nbLas0++;
    }
    if (periodicLaser(&buf1,&laserStruct1)){
        lasStrRec1=timeMicros;;
        nbLas1++;
    }


    //laser data pocessing
    if (laserStruct0.thickness && laserStruct1.thickness){ //case of two laser detected
        //return the the best signal (i.e. one with the biggest thickness)
        if (laserStruct0.thickness>laserStruct1.thickness) laserStruct=laserStruct0;
        else laserStruct=laserStruct1;

        //reset
        memset(&laserStruct0,0,sizeof(plStruct));
        memset(&laserStruct1,0,sizeof(plStruct));
    }
    else if (laserStruct0.thickness && (timeMicros-lasStrRec0)>(laser_period>>4)){ //one laser detected and we don't expect the other one anymore (one eight of the period later) FIXME : use measured period
//        if ((timeMicros-lastLaserDetectMicros)>(laser_period>>2))
            laserStruct=laserStruct0;
        memset(&laserStruct0,0,sizeof(plStruct));
    }
    else if (laserStruct1.thickness && (timeMicros-lasStrRec1)>(laser_period>>4)){ //one laser detected and we don't expect the other one anymore (one eight of the period later) FIXME : use measured period
//        if ((timeMicros-lastLaserDetectMicros)>(laser_period>>2))
            laserStruct=laserStruct1;
        memset(&laserStruct1,0,sizeof(plStruct));
    }

    if ( laserStruct.thickness ) {
        lastLaserDetectMicros=laserStruct.date;
        lastLaserDetectMillis=laserStruct.date/1000;
    }


//STATE MACHINE
    switch (state){
        case S_BEGIN :
            if (rxB && inMsg.header.type==E_SYNC_DATA && inMsg.payload.sync.flag==SYNCF_BEGIN_ELECTION){
                state=S_SYNC_ELECTION;
                printf("begin election");
            }
            else break;
        case S_SYNC_ELECTION :
            if (prevState!=state) {
                nbLas0=0;
                nbLas1=0;
                prevState=state;
            }
            // Determine the best laser interruption to perform the synchronization (the one with the highest count during syncIntSelection)
            if (rxB && inMsg.header.type==E_SYNC_DATA && inMsg.payload.sync.flag==SYNCF_MEASURES){
                chosenOne=(nbLas0<nbLas1?1:0);
                bn_printDbg("end election\n");
                state=S_SYNC_MEASURES;
            }
            else {
                break;
            }
        case S_SYNC_MEASURES:
            // laser data (if value is ours for sure (ie comes from a tracked measure XXX check if periodicLaser returns a struct compatible with that)
            if (chosenOne==0 && laserStruct0.thickness && laserStruct0.period){
                syncComputationLaser(&laserStruct0);
            }
            else if(chosenOne==1 && laserStruct1.thickness && laserStruct0.period) {
                syncComputationLaser(&laserStruct1);
            }
            // handling data broadcasted by turret
            if (rxB && inMsg.header.type==E_SYNC_DATA){
                    rxB=0;
                if (inMsg.payload.sync.flag==SYNCF_END_MEASURES){
                    bn_printDbg("syncComputation\n");
//                    syncComputationFinal(&inMsg.payload.sync);
                    state=S_GAME;
                }
                else {
                    //syncComputationMsg(&inMsg.payload.sync);
                }
            }
        	break;
#if 0   // probably not usefull, so skipped.
        case S_SYNCED : // waiting the signal from main to go to game mode
            if (prevState!=state) {
                prevState=state;
            }
            if (rxB && inMsg.header.type==E_SYNC_OK && inMsg.header.srcAddr==ADDRX_MAIN){
                state=S_GAME;
            }

            break;
#endif
        case S_GAME :
            if (prevState!=state) {
                prevState=state;
            }
        	if ( laserStruct.thickness ) { //if there is some data to send
				outMsg.header.destAddr=ADDRX_MAIN;
				outMsg.header.type=E_MEASURE;
				outMsg.header.size=sizeof(sMobileReportPayload);

        	    if (laserStruct.period) outMsg.payload.mobileReport.value=delta2dist(laserStruct.deltaT,laserStruct.period);
        	    else outMsg.payload.mobileReport.value=delta2dist(laserStruct.deltaT,laser_period);
                outMsg.payload.mobileReport.date=laserStruct.date;
                outMsg.payload.mobileReport.precision=laserStruct.precision;
                outMsg.payload.mobileReport.sureness=laserStruct.sureness;
				bn_send(&outMsg);
//        	    bn_printfDbg((char*)"%lu, %lu, mes , %lu, delta ,%lu, thick,%lu\n",laser_period,laserStruct.period,outMsg.payload.measure.value,laserStruct.deltaT,laserStruct.thickness);
          }
          break;
        default : break;
    }
}

