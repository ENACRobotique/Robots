#define PIN_DBG_LED 13



#include "Arduino.h"

#include "lib_int_laser.h"
#include "lib_time.h"
#include "messages.h"
#include "../../../communication/botNet/shared/botNet_core.h"
#include "params.h"
#include "network_cfg.h"
#include "MemoryFree.h"
#include "../../../communication/botNet/shared/bn_debug.h"


char nbSync=0;
unsigned long lastLaserDetectMillis=0,lastLaserDetectMicros=0;
int debug_led=1;
unsigned long time_prev_led=0, time_prev_laser=0;
int state=GAME;
plStruct laserStruct0={0},laserStruct1={0};
unsigned long lasStrRec0=0,lasStrRec1=0;
volatile unsigned long laser_period=100000; // in µs, to be confirmed by the main robot



inline void periodHandle(sMsg *msg){
    if (msg->header.type==E_PERIOD)  laser_period=msg->payload.period;
}





void setup() {
  laserIntInit(0);
  laserIntInit(1);

  pinMode(13,OUTPUT);

  bn_init();

  bn_printDbg("start mobile 1");
  bn_attach(E_DEBUG_SIGNALLING,&bn_debugUpdateAddr);
  bn_attach(E_PERIOD,&periodHandle);
}

int routineErr=0,i=0;
unsigned long sw=0;
int led=1;
sMsg in;

void loop() {

    sMsg inMsg={{0}},outMsg={{0}};
    plStruct laserStruct={0}; //structure containing info about the last laser detection (deleted if not processed within one loop)
    int rxB=0; // size (bytes) of message available to read
    unsigned long time = millis(),timeMicros=micros();

    //blink
    if((time - time_prev_led)>=3000) {
      time_prev_led= time;
      digitalWrite(PIN_DBG_LED,debug_led^=1);
#ifdef DEBUG
      bn_printfDbg((char*)"%lu, mem : %d\n",time/1000,freeMemory());
#endif
    }

//MUST ALWAYS BE DONE (any state)


    // routine and receive
    if ((rxB=bn_receive(&inMsg))) {
        i++;
    }
    //reading the eventual data from the lasers
    if (periodicLaser(&buf0,&laserStruct0)) lasStrRec0=timeMicros;
    if (periodicLaser(&buf1,&laserStruct1)) lasStrRec1=timeMicros;;


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
#if 1
    switch (state){
        case SYNC:
        	if (rxB){
        		switch (inMsg.header.type) {
        		case E_SYNC_EXPECTED_TIME :
					if (nbSync<3){
						if ( abs((l2gMicros(lastLaserDetectMillis)-inMsg.payload.syncTime))<SYNC_TOL ){
							nbSync++;
						}
						else {
							setMicrosOffset(lastLaserDetectMillis-inMsg.payload.syncTime);//FIXME : to correct
							nbSync=0; //TODO : man, i'm no sure about this one. I is highly unprobable that we receive 3 times a laser in sync with our TXed time. On the other hand, we will miss some message (or unsync)
						}
					}
					if (nbSync>=3){//yeah, I know, but fuck you, I have my reason (ensures that the syncOK messages are correctly received)
						outMsg.header.destAddr=ADDRX_MAIN;
						outMsg.header.type=E_SYNC_OK;
						outMsg.header.size=0;
						bn_send(&outMsg);
					}
					rxB=0;
					break;
        		case E_SYNC_OK :
        			//switch to game state if the sync_ok signal was send by the main
        			if ( (inMsg.header.srcAddr == ADDRX_MAIN) || inMsg.header.srcAddr == ADDRI_MAIN_TURRET){
					  state=GAME;
					}
        			rxB=0;
        			break;
        		default : break;
        		}
			}
        	break;


        case GAME :
        	if ( laserStruct.thickness ) { //if there is some data to send
//				outMsg.header.destAddr=ADDRX_MAIN;
//				outMsg.header.type=E_MEASURE;
//				outMsg.header.size=sizeof(sMesPayload);
//
//                outMsg.payload.measure.value=laserStruct.deltaT;
////                if (laserStruct.period) outMsg.payload.measure.value=laser2dist(laserStruct.deltaT,laserStruct.period);
////                else outMsg.payload.measure.value=laser2dist(laserStruct.deltaT,laser_period);
//                outMsg.payload.measure.date=laserStruct.date;
//                outMsg.payload.measure.precision=laserStruct.precision;
//                outMsg.payload.measure.sureness=laserStruct.sureness;
//				bn_send(&outMsg);
        	    bn_printfDbg((char*)"date, %lu, mes : %lu\n",laserStruct.date,delta2dist(laserStruct.period,laserStruct.deltaT));
          }
          break;
        default : break;
    }
#endif
}

