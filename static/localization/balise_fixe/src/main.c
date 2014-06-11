
#include <absolutepos.h>
#include <driverlib/fpu.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <inc/hw_memmap.h>
#include <lib_int_laser.h>
#include <lib_synchro_beacon.h>
#include <messages.h>
#include <params.h>
#include <perception.h>
#include <roles.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "../../../communication/botNet/shared/botNet_core.h"
#include "../../../communication/botNet/shared/message_header.h"
#include "../../../communication/network_tools/bn_debug.h"

#ifndef BIT
#define BIT(a) (1<<(a))
#endif

#define MEAS_BUF_SIZE 8
sMeasures measuresBuf[MEAS_BUF_SIZE]={{0}};
int measuresIndex=0,prevMeasuresIndex=0;
plStruct stat_tempPl;

void pushMeasure(plStruct *pl,eBeacon beacon){
    measuresBuf[measuresIndex].beacon=beacon;
    measuresBuf[measuresIndex].date=pl->date;
    measuresBuf[measuresIndex].deltaT=pl->deltaT;
    measuresBuf[measuresIndex].period=pl->period;
    measuresBuf[measuresIndex].u_date=pl->precision;

    measuresIndex=(measuresIndex + 1 ) % MEAS_BUF_SIZE;
}

void __error__(char *pcFilename, unsigned long ulLine){
    while(1);
}

uint32_t laser_period=50000;
uint32_t lasCount[LAS_INT_TOTAL]={0};       // sum of all laser interruption thickness detected on channel n
int chosenOne=0;                           // interruption chosen for synchronization (with highest sum of all thicknesses)

inline void periodHandle(sMsg *msg){
    if (msg->header.type==E_PERIOD)  laser_period=msg->payload.period;
}

// main function.
int main(void) {
    unsigned char light =0x04,led=0;
    static uint32_t prevLed=0;
    int ret;

	FPUEnable();

	timerInit();

	// Initialisation

    // Enable the GPIO port that is used for the on-board LEDs.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Enable the GPIO pins as output for the LEDs (PF1-3).
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2);


    bn_attach(E_ROLE_SETUP,role_setup);
//    bn_attach(E_PERIOD,&periodHandle);


    if ((ret=bn_init())<0){
        light=0x02;
    }

    bn_printDbg("fixed start\n");
//    bn_printfDbg("start fixed, addr %hx\n",MYADDR);

    laserIntInit();

    //mainState state=S_SYNC_ELECTION, prevState=S_BEGIN; fixme todo xxx
    mainState state=S_GAME, prevState=S_BEGIN;

/*********************** loop ************************/
    while(1){
        plStruct plTable[LAS_INT_TOTAL]={{0}};
        sMsg inMsg={{0}};

        int rxB=bn_receive(&inMsg);

        updateSync();

        if ((millis()-prevLed)>1000){
            prevLed=millis();
            led^=light;
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2,led);
#ifdef DEBUG
//            bn_printfDbg("%lu stellaris blink",millis());
#endif
        }

        // reading eventual new values
        int j;
        for (j=0; j<LAS_INT_TOTAL; j++ ){

            if (ildTable[j].deltaT && newLaserMeasure(&ildTable[j],&plTable[j])){
//                bn_printfDbg("int %d, dt %lu th %lu per %lu",j,plTable[j].deltaT,plTable[j].thickness,plTable[j].period);

                switch (j){
                case LAS_INT_0 :
                    pushMeasure(&plTable[j],BEACON_2);
                    break;
                case LAS_INT_1 :
                    pushMeasure(&plTable[j],BEACON_1);
                    break;
                case LAS_INT_2 :
                case LAS_INT_3 :
                    if ( stat_tempPl.deltaT && (plTable[j].date-stat_tempPl.date)< (laser_period>>5)){
                        if (plTable[j].thickness<stat_tempPl.thickness){
                            pushMeasure(&stat_tempPl,BEACON_3);

                        }
                        else {
                            pushMeasure(&plTable[j],BEACON_3);
                        }
                        memset(&stat_tempPl,0,sizeof(stat_tempPl));
                    }
                    else{
                        stat_tempPl=plTable[j];
                    }
                    break;
                default : break;
                }
                lasCount[j]+=plTable[j].thickness;
                ildTable[j].deltaT=0;
            }
        }
        if ( stat_tempPl.deltaT && (micros()-stat_tempPl.date)>(laser_period>>5)){
            pushMeasure(&stat_tempPl,BEACON_3);
            memset(&stat_tempPl,0,sizeof(stat_tempPl));
        }

        //STATE MACHINE
        switch (state){
            case S_BEGIN :
                if (rxB && inMsg.header.type==E_SYNC_DATA && inMsg.payload.sync.flag==SYNCF_BEGIN_ELECTION){
                    state=S_SYNC_ELECTION;
#ifdef VERBOSE_SYNC
                    bn_printDbg("begin election");
#endif
                }
                else break;
                /* no break */
            case S_SYNC_ELECTION :
                if (prevState!=state) {
                    // reset counters
                    memset(lasCount,0,sizeof(lasCount));
                    prevState=state;
                }
                // Determine the best laser interruption to perform the synchronization (the one with the highest count during syncIntSelection)
                if (rxB && inMsg.header.type==E_SYNC_DATA && inMsg.payload.sync.flag==SYNCF_MEASURES){
                    int k;
                    int maxTh=0;
                    for (k=0; k<LAS_INT_TOTAL; k++){
                        if (lasCount[k]>maxTh){
                            maxTh=lasCount[k];
                            chosenOne=k;
                        }
                    }
#ifdef VERBOSE_SYNC
                    bn_printDbg("end election\n");
#endif
                    state=S_SYNC_MEASURES;
                }
                else {
                    break;
                }
                /* no break */
            case S_SYNC_MEASURES:
                // laser data (if value is ours for sure (ie comes from a tracked measure)
                if ( plTable[chosenOne].thickness && plTable[chosenOne].period){
                    syncComputationLaser(&plTable[chosenOne]);
                }
                // handling data broadcasted by turret
                if (rxB && inMsg.header.type==E_SYNC_DATA){
                        rxB=0;
                    if (inMsg.payload.sync.flag==SYNCF_END_MEASURES){
#ifdef VERBOSE_SYNC
                        bn_printDbg("syncComputation\n");
#endif
                        syncComputationFinal(&inMsg.payload.sync);
                        state=S_GAME;
                    }
                    else {
                        syncComputationMsg(&inMsg.payload.sync);
                    }
                }
                break;
            case S_GAME :
                if (((MEAS_BUF_SIZE+measuresIndex-prevMeasuresIndex)%MEAS_BUF_SIZE)>=1){
#ifdef DEBUG
                    int k;
                    for (k=0; k < (MEAS_BUF_SIZE+measuresIndex-prevMeasuresIndex)%MEAS_BUF_SIZE ; k++){
                        int tempindex=(prevMeasuresIndex+k)%MEAS_BUF_SIZE;
//                        bn_printfDbg("t %lu beac %d, dt %lu per %lu",measuresBuf[tempindex].date,measuresBuf[tempindex].beacon,measuresBuf[tempindex].deltaT,measuresBuf[tempindex].period);
                    }
#endif
                    absolutepos(measuresBuf,measuresIndex,MEAS_BUF_SIZE);
                    prevState=state;
                }
              break;
            default : break;
        }//switch

        prevMeasuresIndex=measuresIndex;
    } // while 1
}

