#define PIN_DBG_LED 13
#define PIN_RST_XBEE 5


#include "Arduino.h"

#include "lib_int_laser.h"
#include "lib_time.h"
#include "messages.h"
#include "lib_superBus.h"
#include "params.h"
#include "network_cfg.h"



char nbSync=0;
unsigned long lastLaserDetect=0,lastLaserDetectMicros=0;
int debug_led=1;
unsigned long time_prev_led=0, time_prev_laser=0;
int state=GAME;
plStruct laserStruct0={0},laserStruct1={0};
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



}

void loop() {


    sMsg inMsg={{0}},outMsg={{0}};
    plStruct laserStruct2Send={0};
    int rxB=0; // size (bytes) of message available to read
    int las0=0,las1=0; // boolean, saying if laser 0 or 1 have received something in this loop
    unsigned long time = millis(),timeMicros=micros();



//MUST ALWAYS BE DONE (any state)

	//network routine and test if message for this node
    sb_routine();
	rxB=sb_receive(&inMsg);

    //reading the eventual data from the lasers
    las0=periodicLaser(&buf0,&laserStruct0);
    las1=periodicLaser(&buf1,&laserStruct1);

    //previous data pocessing
    //if some laser has been detected on 0
    if (las0){
        if ( (laserStruct0.date-lastLaserDetect)> (laser_period>>1) ) { //more than half the period after the last laser detected
            lastLaserDetect=laserStruct0.date;
            lastLaserDetectMicros=micros();
        }
        else if ((laserStruct0.date-lastLaserDetect)< (laser_period>>6)){ //if it is the second detection of the same turn that have already been recorded in lastLaserDetecd
            //test thickness and write lastLaseDetect and laserStruct2Send
            if (laserStruct0.thickness > laserStruct1.thickness){ //"returns" the value with the biggest thickness
                memcpy(&laserStruct2Send,&laserStruct0,sizeof(plStruct));
                lastLaserDetect=laserStruct0.date;
            }
            else {
                memcpy(&laserStruct2Send,&laserStruct1,sizeof(plStruct));
                lastLaserDetect=laserStruct1.date;
            }
            //reset everything
            memset(&laserStruct0,0,sizeof(plStruct));
            memset(&laserStruct1,0,sizeof(plStruct));
        }
    }
    //if some laser has been detected on 1
    if (las1){
        //more than half the period after the last laser detected
        if ( (laserStruct1.date-lastLaserDetect)> (laser_period>>1)) {
            lastLaserDetect=laserStruct0.date;
            lastLaserDetectMicros=micros();
        }
        //if it is the second detection of the same turn that have already been recorded in lastLaserDetecd
        else if ((laserStruct1.date-lastLaserDetect)< (laser_period>>6)){
            //test thickness and write lastLaseDetect and laserStruct2Send
            if (laserStruct0.thickness > laserStruct1.thickness){
                //"returns" the value with the biggest thickness
                memcpy(&laserStruct2Send,&laserStruct0,sizeof(plStruct));
                lastLaserDetect=laserStruct0.date;
            }
            else {
                memcpy(&laserStruct2Send,&laserStruct1,sizeof(plStruct));
                lastLaserDetect=laserStruct1.date;
            }
            //reset everything
            memset(&laserStruct0,0,sizeof(plStruct));
            memset(&laserStruct1,0,sizeof(plStruct));
        }
    }
    //if we do not expect the second laser detection anymore
    if ( (!las0 && !las1) && (timeMicros-lastLaserDetectMicros)> (laser_period>>6) && (laserStruct0.date || laserStruct1.date) ){
        //writes laserStruct2Send
        if (laserStruct0.thickness > laserStruct1.thickness){ //"returns" the value with the biggest thickness
            memcpy(&laserStruct2Send,&laserStruct0,sizeof(plStruct));
        }
        else memcpy(&laserStruct2Send,&laserStruct1,sizeof(plStruct));
        //reset everything
        memset(&laserStruct0,0,sizeof(plStruct));
        memset(&laserStruct1,0,sizeof(plStruct));
    }

    //update the laser period
    if (rxB && inMsg.header.type==E_PERIOD ){
    	laser_period=inMsg.payload.period;
    	rxB=0;

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
							setMicrosOffset(lastLaserDetect-inMsg.payload.syncTime);//FIXME : to correct
							nbSync=0; //TODO : man, i'm no sure about this one. I is highly unprobable that we receive 3 times a laser in sync with our TXed time. On the other hand, we will miss some message (or unsync)
						}
					}
					if (nbSync>=3){//yeah, I know, but fuck you, I have my reason (ensures that the syncOK messages are correctly received)
						outMsg.header.destAddr=ADDRX_MAIN;
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
        	if ( laserStruct2Send.date ) { //if there is some data to send
				outMsg.header.destAddr=ADDRX_MAIN;
				outMsg.header.type=E_MEASURE;
				outMsg.header.size=sizeof(sMesPayload);

                outMsg.payload.measure.value=laserStruct2Send.deltaT;
                outMsg.payload.measure.date=laserStruct2Send.date;
                outMsg.payload.measure.precision=laserStruct2Send.precision;
                outMsg.payload.measure.sureness=laserStruct2Send.sureness;
				sb_send(&outMsg);
          }
          break;
        default : break;

    }
}

