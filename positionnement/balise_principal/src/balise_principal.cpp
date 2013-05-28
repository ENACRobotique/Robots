/*
 * balise_principal.cpp
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#include "lib_domitille.h"


#include "lib_xbee_arduino.h"
#include "params.h"
#include "Arduino.h"

#include "Wire.h"
#include "i2ccomm.h"
#include "inttypes.h"

extern "C"{
    #include "messages.h"
    #include "lib_comm.h"
}
uint32_t mesTab[2]={0,0};



void setup(){
    Serial.begin(111111);

    pinMode(PIN_DBG_LED,OUTPUT);
    pinMode(PIN_RST_XBEE,OUTPUT);



    //setupXbee();

    digitalWrite(PIN_RST_XBEE,HIGH);
    delay(100);
    digitalWrite(PIN_RST_XBEE,LOW);
    delay(200);

    while(Serial.read() != -1); //flush the buffer for any incorrect bytes


    Wire.begin(8);
    Wire.onRequest(&requestHandler);

    domi_init(2);

#ifdef DEBUG
    debugMsg pkt={
          ADDR_DEBUG,
          MYADDR,
          E_DEBUG,
          (ADDR_DEBUG+MYADDR+E_DEBUG),
          668,
          0,
          "demarrage tourelle"
    };
    Serial.write((unsigned char*)&pkt, sizeof(debugMsg));
#endif



}


static int state=GAME,debug_led=0;
char syncOKbool=0;

void loop(){
    uMsg incMsg;

    unsigned int sizeIncoming=0;
    static unsigned long time_prev_led=0,time_prev_period=millis(),prev_TR=0;
    unsigned long time=millis();


    //periodically send the period in broadcast
    if((time - time_prev_period)>=ROT_PERIOD_BCAST) {
        time_prev_period += ROT_PERIOD_BCAST;
        periodMsg pkt={
                ADDR_BROADCAST,
                MYADDR,
                E_PERIOD,
                (ADDR_BROADCAST+MYADDR+E_PERIOD)%255,
                domi_meanPeriod()
        };
    Serial.write((unsigned char*)&pkt,sizeof(periodMsg));
    }


    //reading an eventual incoming message
    if ( readXbee(&incMsg)!=-1 ){

        //check if destination says it is for us and checksum
        if ((MYADDR & incMsg.header.destAddr) ){
          switch(incMsg.header.type){

              //if the sync has succeed for one node
              case E_SYNC_OK :{
                          syncOKbool|=incMsg.syncOk.srcAddr;
                            #ifdef DEBUG
                                  debugMsg pkt={ ADDR_DEBUG, MYADDR, E_DEBUG,
                                            (ADDR_DEBUG+MYADDR+E_DEBUG),0,0,"sync received from"};
                                  pkt.i=incMsg.syncOk.srcAddr;
                                  Serial.write((unsigned char*)&pkt, sizeof(debugMsg));
                            #endif
                          if (syncOKbool==(ADDR_MOBILE_1)){// | ADDR_MOBILE_2)){ //FIXME change if more beacon to sync
                              syncOkMsg msg={ ADDR_BROADCAST, MYADDR, E_SYNC_OK,
                                        (ADDR_BROADCAST+MYADDR+E_SYNC_OK)};
                              Serial.write((unsigned char*)&msg, sizeof(syncOkMsg));
                              state=GAME;
                          }
                          break;
              }
              case E_MEASURE :{
                                if (incMsg.measure.srcAddr==ADDR_MOBILE_1) {
                                    static unsigned long last1[10]={0};
                                    static int i1=0;
                                    int i=0;
                                    unsigned long max=0;

                                    i1=(i1+1)%10;
                                    last1[i1]=incMsg.measure.measure;
                                    for (i=0;i<10;i++){
                                        if(last1[i]>max) max=last1[i];
                                    }
                                    mesTab[0]=max;
                                }
                                else if (incMsg.measure.srcAddr==ADDR_MOBILE_2) {
                                    static unsigned long last2[10]={0,0,0,0,0,0,42,0,0,0};
                                    static int i2=0;
                                    int i=0;
                                    unsigned long max=0;

                                    i2=(i2+1)%10;
                                    last2[i2]=incMsg.measure.measure;
                                    for (i=0;i<10;i++){
                                        if(last2[i]>max) max=last2[i];
                                    }
                                    mesTab[1]=max;
                                }

                                break;
              }
              default :
                          break;
          }
        }
    }


      switch (state){
          case SYNC:{
                  //if nouveau tour, envoyer les expected pour le dernier tour
                  if(prev_TR!=last_TR){
                      //expected for balise 1
                      syncDataMsg pkt={
                              ADDR_MOBILE_1,
                              ADDR_MAIN,
                              E_SYNC_EXPECTED_TIME,
                              (ADDR_MOBILE_1+ADDR_MAIN+E_SYNC_EXPECTED_TIME),
                              ((TR_period*PHASE_INIT_MOBILE_1)>>9)+prev_TR
                      };
                      //if the destination's syncOK has not been received
                      if (!(syncOKbool & ADDR_MOBILE_1)) Serial.write((unsigned char*)&pkt,sizeof(syncDataMsg));
//                      if (!(syncOKbool & ADDR_MOBILE_2)){  FIXME
//                          pkt.destAddr=ADDR_MOBILE_2;
//                          pkt.checksum=checksum((unsigned char*)&pkt);
//                          pkt.time=((TR_period*PHASE_INIT_MOBILE_2)>>9)+prev_TR;
//                          Serial.write((unsigned char*)&pkt,sizeof(syncDataMsg));
//                      }
                      prev_TR=last_TR;
                  }

                  break;
          }
          case GAME :{
                  break;
          }
          default : break;

      }


      if((time - time_prev_led)>=500) {
        time_prev_led += 500;
        digitalWrite(PIN_DBG_LED,debug_led^=1);
      }


}
