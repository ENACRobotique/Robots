#define PIN_DBG_LED 13
#define PIN_RST_XBEE 5


#include "Arduino.h"

#include "lib_int_laser.h"
#include "lib_time.h"
#include "messages.h"
#include "lib_superBus.h"
#include "params.h"
#include "network_cfg.h"





int debug_led=1;
unsigned long time=0, time_prev_led=0, time_prev_laser=0;
int state=GAME;
plStruct laserStruct0,laserStruct1;
unsigned long laser_period=50000; // in µs, to be confirmed by the main robot





void setup() {
  laserIntInit(0);
  laserIntInit(1);
  Serial.begin(111111);

  pinMode(PIN_DBG_LED,OUTPUT);
  pinMode(PIN_RST_XBEE,OUTPUT);

  digitalWrite(PIN_RST_XBEE,HIGH);
  delay(100);
  digitalWrite(PIN_RST_XBEE,LOW);
  delay(200);

#ifdef DEBUG
  sb_printDbg(ADDRX_DEBUG,"mobile starting",-16,13);
#endif
}

void loop() {
    static char nbSync=0;
    static unsigned long lastLaserDetect=0;
    sMsg inMsg={{0}},outMsg={{0}};
    int rxB=0; // size (bytes) of message available to read
    time = millis();



//MUST ALWAYS BE DONE (any state)

	//network routine and test if message for this node
	if (sb_routine()){
		rxB=sb_receive(&inMsg);
	}
    //reading the eventual data from the lasers
    laserStruct0=periodicLaser(&buf0);
    laserStruct1=periodicLaser(&buf1);

    if (laserStruct0.date || laserStruct1.date){ //if there is a new value
        if (laserStruct0.thickness > laserStruct1.thickness) lastLaserDetect=laserStruct0.date;
        else lastLaserDetect=laserStruct1.date;
    }

    //update the laser period
    if (rxB && inMsg.header.type==E_PERIOD ){
    	laser_period=inMsg.payload.period;
    	rxB=0;
#ifdef DEBUG
    	sb_printDbg(ADDRX_DEBUG,"period received",0,inMsg.payload.period);
#endif
    }

    //blink
    if((time - time_prev_led)>=500) {
      time_prev_led= time;
      digitalWrite(PIN_DBG_LED,debug_led^=1);
    }

//STATE MACHINE
    switch (state){
        case SYNC:
        	if (rxB){
        		switch (inMsg.header.type) {
        		case E_SYNC_EXPECTED_TIME :
					if (nbSync<3){
						if ( abs((l2gMicros(lastLaserDetect)-inMsg.payload.syncTime))<SYNC_TOL ){
							nbSync++;
						}
						else {
							setMicrosOffset(lastLaserDetect-inMsg.payload.syncTime);
							nbSync=0; //TODO : man, i'm no sure about this one. I is highly unprobable that we receive 3 times a laser in sync with our TXed time. On the other hand, we will miss some message (or unsync)
						}
					}
					if (nbSync>=3){//yeah, I know, but fuck you, I have my reason (ensures that the syncOK messages are correctly received)
						outMsg.header.destAddr=ADDRX_MAIN;
						outMsg.header.srcAddr=MYADDRX;
						outMsg.header.type=E_SYNC_OK;
						outMsg.header.size=0;
						sb_send(&outMsg);
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
        	if (laserStruct0.deltaT || laserStruct1.deltaT){ //if there is a new value, sends it to the main
				outMsg.header.destAddr=ADDRX_MAIN;
				outMsg.header.srcAddr=MYADDRX;
				outMsg.header.type=E_MEASURE;
				outMsg.header.size=sizeof(sMesPayload);
				if (laserStruct0.thickness > laserStruct1.thickness) {
					outMsg.payload.measure.value=laserStruct0.deltaT;
					outMsg.payload.measure.date=laserStruct0.date;
					outMsg.payload.measure.precision=laserStruct0.precision;
					outMsg.payload.measure.sureness=laserStruct0.sureness;
				}
				else {
					outMsg.payload.measure.value=laserStruct1.deltaT;
					outMsg.payload.measure.date=laserStruct1.date;
					outMsg.payload.measure.precision=laserStruct1.precision;
					outMsg.payload.measure.sureness=laserStruct1.sureness;
				}
				sb_send(&outMsg);
          }
          break;
        default : break;

    }
}

