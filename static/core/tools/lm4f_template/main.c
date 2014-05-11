/*
* Copyright (c) 2012, Mauro Scomparin
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Mauro Scomparin nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY Mauro Scomparin ``AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL Mauro Scomparin BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* File:         main.c.
* Author:       Mauro Scomparin <http://scompoprojects.worpress.com>.
* Version:      1.0.0.
* Description:  Main sample file.
*/

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

#define DEBOUNCE_DELAY 10

void sw1Interrupt(){
    static int state=0xff;
    GPIOPinIntClear(GPIO_PORTF_BASE,GPIO_PIN_4); // must be done "early", takes several cycle to be accomplished, cf doc)
    delay(DEBOUNCE_DELAY); //debounce

    if (!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)) {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,state);
        state^=0xff;
    }
}

void sw2Interrupt(){
    static int state=0;
    GPIOPinIntClear(GPIO_PORTF_BASE,GPIO_PIN_0); // must be done "early", takes several cycle to be accomplished, cf doc)
    delay(DEBOUNCE_DELAY);

    if (!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0))  {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,state);
        state^=0xff;
    }

}

void portFIntHandler(){
    long read=GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);
    if ( !(read & GPIO_PIN_4) ) sw1Interrupt();
    if ( !(read & GPIO_PIN_0) ) sw2Interrupt();
}

// main function.
int main(void) {
    volatile unsigned long ulLoop;

    timerInit();

    // Enable the GPIO port that is used for the on-board LEDs.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Delay for a bit (cf previous function documentation)
    for(ulLoop = 0; ulLoop < 5; ulLoop++);


    // Enable the GPIO pins as output for the LEDs (PF1-3).
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);


    // Hack to enable the use of PF0 : (from http://codeandlife.com/2012/10/16/ti-stellaris-launchpad-test-run/ )
    // Unlock PF0 so we can change it to a GPIO input
    // Once we have enabled (unlocked) the commit register then re-lock it
    // to prevent further changes.  PF0 is muxed with NMI thus a special case.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // Enable as input the GPIO pins for switch 1 & 2 (PF4 et PF0)
    GPIOPinTypeGPIOInput( GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);

    // Set up pull-up resistors
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    // Set them as interrupt trigerring, on rising edge
    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_FALLING_EDGE);

    // Register the interrupt handler for port F
    GPIOPortIntRegister(GPIO_PORTF_BASE,portFIntHandler);

    // Enable the interrupts
    GPIOPinIntEnable(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);


    //
    // Loop forever.
    //
    while(1)
    {

#define DUR 100
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0x02);

        delay(DUR);

        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0x00);

        delay(DUR);
    }

}
