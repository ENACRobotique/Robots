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

#include <stdlib.h>

#ifndef BIT
#define BIT(a) (1<<a)
#endif

void __error__(char *pcFilename, unsigned long ulLine){
    while(1);
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

    if ((ret=bn_init())<0){
        light=0x02;
    }


    laserIntInit();

    // Loop forever.
    while(1){
        bn_receive(NULL);

        if ((millis()-prevLed)>1000){
            prevLed=millis();
            led^=light;
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2,led);
            bn_printDbg("stellaris blink");
        }


    }

}
