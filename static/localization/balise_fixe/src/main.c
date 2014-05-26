#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "time.h"
#include "../../communication/botNet/shared/botNet_core.h"
#include "../../communication/network_tools/bn_debug.h"
#include "roles.h"
#include "inc/lm4f120h5qr.h"

#include "lib_int_laser.h"
#include "lib_synchro_beacon.h"

#include "params.h"

#include <stdlib.h>

#ifndef BIT
#define BIT(a) (1<<a)
#endif

void __error__(char *pcFilename, unsigned long ulLine){
    while(1);
}

uint32_t laser_period=0;

inline void periodHandle(sMsg *msg){
    if (msg->header.type==E_PERIOD)  laser_period=msg->payload.period;
}


// main function.
int main(void) {
    unsigned char light =0x04,led=0;
    static uint32_t prevLed=0;
    int ret;

    timerInit();

    // Enable the GPIO port that is used for the on-board LEDs.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Enable the GPIO pins as output for the LEDs (PF1-3).
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2);


    bn_attach(E_ROLE_SETUP,role_setup);
    bn_attach(E_PERIOD,&periodHandle);
    bn_printfDbg("start fixed, addr %hx\n",MYADDR);

    if ((ret=bn_init())<0){
        light=0x02;
    }


    laserIntInit();

    // Loop forever.
    mainState state=S_SYNC_ELECTION, prevState=S_BEGIN;

/*********************** loop ************************/
    while(1){
        sMsg inMsg={{0}};

        int rxB=bn_receive(&inMsg);

        updateSync();

        if ((millis()-prevLed)>1000){
            prevLed=millis();
            led^=light;
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2,led);
#ifdef DEBUG
            bn_printfDbg("%lu stellaris blink",millis());
#endif
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
//                    intLas0=0;
//                    intLas1=0;
                    prevState=state;
                }
                // Determine the best laser interruption to perform the synchronization (the one with the highest count during syncIntSelection)
                if (rxB && inMsg.header.type==E_SYNC_DATA && inMsg.payload.sync.flag==SYNCF_MEASURES){
//                    chosenOne=(intLas0<intLas1?1:0);
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
//                if (chosenOne==0 && laserStruct0.thickness && laserStruct0.period){
//                    syncComputationLaser(&laserStruct0);
//                }
//                else if(chosenOne==1 && laserStruct1.thickness && laserStruct1.period) {
//                    syncComputationLaser(&laserStruct1);
//                }
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
                //fixme call the nelder-mead.
              break;
            default : break;
        }//switch
        prevState=state;



    } // while 1

}
