#define PIN_DBG_LED 13
#define PIN_RST_XBEE 5


#include "Arduino.h"

#include "lib_int_laser.h"
#include "lib_time.h"
#include "lib_xbee_arduino.h"
#include "messages.h"
#include "lib_comm.h"
#include "params.h"




int debug_led=1;
unsigned long time, time_prev_led=0, time_prev_laser=0;
int state=GAME;
plStruct laserStruct0,laserStruct1;
unsigned long laser_period=50000; // in µs, to be confirmed by the main robot





void setup() {
  laserIntInit(0);
  laserIntInit(1);
  Serial.begin(111111);

  setupXbee();


  pinMode(PIN_DBG_LED,OUTPUT);
  pinMode(PIN_RST_XBEE,OUTPUT);

  digitalWrite(PIN_RST_XBEE,HIGH);
  delay(100);
  digitalWrite(PIN_RST_XBEE,LOW);
  delay(200);


#ifdef DEBUG
  debugMsg pkt={
          ADDR_DEBUG,
          MYADDR,
          E_DEBUG,
          (ADDR_DEBUG+MYADDR+E_DEBUG),
          MYADDR,
          0,
          " : demarrage mobile"
  };
  Serial.write((unsigned char*)&pkt, sizeof(debugMsg));
#endif
}





void loop() {
    unsigned int sizeIncoming=0;
    static char nbSync=0;
    static unsigned long lastLaserDetect=0;
    sMsg incMsg,outMsg;
    int lus;




    time = millis();


//MESSAGE HANDLING
    if ( (lus=rxXbee(&incMsg))!=-1 )
    {   //reading

        if ( (MYADDR & incMsg.header.destAddr) ){ //check destination address
            switch (incMsg.header.type) {
                case E_PERIOD :
                        laser_period=incMsg.payload.period;
                        break;
                case E_SYNC_EXPECTED_TIME : //updates the expected time
                        if (state==SYNC){

                          if (nbSync<3){
                              if ( abs((l2gMicros(lastLaserDetect)-incMsg.payload.syncTime))<SYNC_TOL ){
                                  nbSync++;
                              }
                              else {
                                  setMicrosOffset(lastLaserDetect-incMsg.payload.syncTime);
                                  nbSync=0; //TODO : man, i'm no sure about this one. I is highly unprobable that we receive 3 times a laser in sync with our TXed time. On the other hand, we will miss some message (or unsync)
                              }
                          }
                          if (nbSync>=3){//yeah, I know, but fuck you, I have my reason (ensures that the syncOK messages are correctly received)
                              outMsg.header.destAddr=ADDR_MAIN;
                              outMsg.header.srcAddr=MYADDR;
                              outMsg.header.type=E_SYNC_OK;
                              outMsg.header.size=0;
                              setSum(&outMsg);
                              txXbee(outMsg);
                          }
                        }
                        break;
                case E_SYNC_OK :
                        //if we receive a syncOk from the main, switch to "game" state
                        if (incMsg.header.srcAddr & ADDR_MAIN){
                          state=GAME;
                        }
break;
                default : break;
            }
        }
//        else { //discard message : do nothing
//        }
    }



//MUST ALWAYS BE DONE (any state)
    //reading the eventual data from the lasers
    laserStruct0=periodicLaser(&buf0);
    laserStruct1=periodicLaser(&buf1);

    if (laserStruct0.date || laserStruct1.date){ //if there is a new value
        if (laserStruct0.thickness > laserStruct1.thickness) lastLaserDetect=laserStruct0.date;
        else lastLaserDetect=laserStruct1.date;
    }

//STATE MACHINE for non-"received message" handling functions
    switch (state){
        case SYNC:
            //here, we are waiting for a sync_ok message, action dealt in the rxed message handling par
            break;
        case GAME :
          if (laserStruct0.deltaT || laserStruct1.deltaT){ //if there is a new value, sends it to the main
              outMsg.header.destAddr=ADDR_MAIN;
              outMsg.header.srcAddr=MYADDR;
              outMsg.header.type=E_MEASURE;
              outMsg.header.size=sizeof(sMesPayload);
              if (laserStruct0.thickness > laserStruct1.thickness) {
                  //pkt.dist=laser2dist(laserStruct1.deltaT);
                  outMsg.payload.measure.value=laserStruct0.deltaT;
                  outMsg.payload.measure.date=laserStruct0.date;
                  outMsg.payload.measure.precision=laserStruct0.precision;
                  outMsg.payload.measure.sureness=laserStruct0.sureness;
              }
              else {
                  //pkt.dist=laser2dist(laserStruct1.deltaT);
                  outMsg.payload.measure.value=laserStruct1.deltaT;
                  outMsg.payload.measure.date=laserStruct1.date;
                  outMsg.payload.measure.precision=laserStruct1.precision;
                  outMsg.payload.measure.sureness=laserStruct1.sureness;
                }
          setSum(&outMsg);
          txXbee(outMsg);
          }
          break;
        default : break;

    }


      if((time - time_prev_led)>=500) {
        time_prev_led= time;
        digitalWrite(PIN_DBG_LED,debug_led^=1);
      }


}



