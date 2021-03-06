/*
 * lib_laser_lm4fxx.cpp
 *
 *  Created on: 20 mai 2014
 *      Author: quentin
 */



#ifdef ARCH_LM4FXX

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <inc/hw_memmap.h>
#include "lib_laser_lm4fxx.h"
#include "time.h"

#define BLINK_LASERDETECT


void ld2ilds(bufStruct* bs, ildStruct *ret){
    ldStruct templd;
    templd=laserDetect(bs);
    if (templd.thickness){
        ret->prevDate=ret->date;
        ret->date=templd.date;
        ret->deltaT=templd.deltaT;
        ret->thickness=templd.thickness;
#ifdef BLINK_LASERDETECT
        GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,0x04);
        volatile char ulLoop;
        for( ulLoop = 0; ulLoop < 100; ulLoop++);    // spend several cycle
        GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,0x00);
#endif
    }

}

void portAGPIOIntHandler(){
    static uint32_t db2=0,db3=0,db4=0;
    uint32_t time=micros();
    static long prevRead=0;     // previous state of the pins
    long read=GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4); // get current state of the pins

    long triggering=read ^ prevRead; // pins that triggered the interrupts are the ones for which the state changed.
    prevRead=read;

    // disable the same interrupts to avoid undebouncable interrupts
    GPIOPinIntDisable(GPIO_PORTA_BASE,triggering);
    if ( (triggering & GPIO_PIN_2) ) {
        if ((time-db2)> LASER_DEBOUNCETIME  ){
            db2=time;
            buf[LAS_INT_2].buf[buf[LAS_INT_2].index]=time;
            buf[LAS_INT_2].index++;
            buf[LAS_INT_2].index&=7;
            ld2ilds(&buf[LAS_INT_2],&ildTable[LAS_INT_2]);
        }
    }
    if ( (triggering & GPIO_PIN_3) ) {
        if ((time-db3)> LASER_DEBOUNCETIME  ){
            db3=time;
            buf[LAS_INT_3].buf[buf[LAS_INT_3].index]=time;
            buf[LAS_INT_3].index++;
            buf[LAS_INT_3].index&=7;
            ld2ilds(&buf[LAS_INT_3],&ildTable[LAS_INT_3]);
        }
    }
    if ( (triggering & GPIO_PIN_4) ) {
        if ((time-db4)> LASER_DEBOUNCETIME  ){
            db4=time;
            buf[LAS_INT_1].buf[buf[LAS_INT_1].index]=time;
            buf[LAS_INT_1].index++;
            buf[LAS_INT_1].index&=7;
            ld2ilds(&buf[LAS_INT_1],&ildTable[LAS_INT_1]);
        }
    }

    //clear all the interrupts, plus any that could have appended during the processing
    GPIOPinIntClear(GPIO_PORTA_BASE,0xff);                  // it takes several cycle to be accomplished, cf doc
    volatile char ulLoop;
    for( ulLoop = 0; ulLoop < 3; ulLoop++);    // so we spend several cycle
    //re-enable these interrupts
    GPIOPinIntEnable(GPIO_PORTA_BASE,triggering);
}

void portDGPIOIntHandler(){
    static uint32_t db0=0;
    uint32_t time=micros();
    static long prevRead=0;     // previous state of the pins
    long read=GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0);
    long triggering=read ^ prevRead; // pins that triggered the interrupts are the ones for which the state changed.
    prevRead=read;

    // disable the same interrupts to avoid undebouncable interrupts
    GPIOPinIntDisable(GPIO_PORTD_BASE,triggering);

    if ( (triggering & GPIO_PIN_0) ) {
        if ((time-db0)> LASER_DEBOUNCETIME  ){
            db0=time;
            buf[LAS_INT_0].buf[buf[LAS_INT_0].index]=time;
            buf[LAS_INT_0].index++;
            buf[LAS_INT_0].index&=7;
            ld2ilds(&buf[LAS_INT_0],&ildTable[LAS_INT_0]);
        }
    }
    //clear all the interrupts, plus any that could have appended during the processing
    GPIOPinIntClear(GPIO_PORTD_BASE,0xff);                  // it takes several cycle to be accomplished, cf doc
    volatile char ulLoop;
    for( ulLoop = 0; ulLoop < 30; ulLoop++);    // so we spend several cycle
    //re-enable these interrupts
    GPIOPinIntEnable(GPIO_PORTD_BASE,triggering);
}

/* interrupts :
 *      beacon 1 : PA4 / int 1
 *      beacon 2 : PDO / int 0
 *      beacon 3 : PA2 && PA3 / int 2 & 3
 */
void laser_lm4fxx_init(){

    // Enable the GPIO ports.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Delay for a bit (cf previous function documentation)
    volatile char ulLoop;
    for( ulLoop = 0; ulLoop < 5; ulLoop++);

    // set them as input
    GPIOPinTypeGPIOInput( GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
#ifdef BLINK_LASERDETECT
    GPIOPinTypeGPIOInput( GPIO_PORTD_BASE,GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,0x04);
#else
    GPIOPinTypeGPIOInput( GPIO_PORTD_BASE,GPIO_PIN_0);
#endif

    // Set them as interrupt triggering, on both edges
    GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_BOTH_EDGES);

    // Register the interrupt handlers
    GPIOPortIntRegister(GPIO_PORTA_BASE,portAGPIOIntHandler);
    GPIOPortIntRegister(GPIO_PORTD_BASE,portDGPIOIntHandler);

    // Enable the interrupts
    GPIOPinIntEnable(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
    GPIOPinIntEnable(GPIO_PORTD_BASE,GPIO_PIN_0);
}
void laser_lm4fxx_deinit(){
//todo
}


#endif
