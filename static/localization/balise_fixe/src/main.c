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

#include "inc/lm4f120h5qr.h"

#ifndef BIT
#define BIT(a) (1<<a)
#endif


// main function.
int main(void) {

    timerInit();

    // Loop forever.
    while(1){

    }


}
