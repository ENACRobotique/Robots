/*
 * encoders.c
 *
 *  Created on: 9 févr. 2015
 *      Author: yoyo
 */

#include <lpc214x.h>
#include "param.h"
#include <eint.h>
#include <ime.h>
#include <gpio.h>

#include "encoder.h"

#include "encoders.h"

volatile int irqCpt = 0;
volatile int dir_mes = 0; // clockwise rotation = -1; trigonometry = 1

encoder_t enc1, enc2, enc3;

#define DIR_MES_TRIGO 1
#define DIR_MES_HORAI -1
#define NB_CHANNEL 2
#define RES_CHANNEL 500 // inc/turn

extern volatile int irqCpt;
extern volatile int irqCptRef;
extern volatile int dir_mes;

// EINT0 sur P0,1 => Channel A
void isr_eint0() __attribute__ ((interrupt("IRQ")));
void isr_eint0() {
    SCB_EXTINT = BIT(0); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    if (ChannelB) // B =1
        dir_mes = DIR_MES_TRIGO; // sens = +1
    else
        dir_mes = DIR_MES_HORAI; // sens = -1

    irqCpt = irqCpt + dir_mes;
}

// EINT3 sur P0,20 => Channel B
void isr_eint3() __attribute__ ((interrupt("IRQ")));
void isr_eint3() {
    SCB_EXTINT = BIT(3); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    if (ChannelA)
        dir_mes = DIR_MES_HORAI; // sens = -1
    else
        dir_mes = DIR_MES_TRIGO; // sens = +1

    irqCpt = irqCpt + dir_mes;
}

void encoders_init(){
    encoder_init(&enc1, EINT0, EINT0_P0_16, EINT_RISING_EDGE, isr_eint0, 2);
    encoder_init(&enc2, EINT3, EINT3_P0_20, EINT_RISING_EDGE, isr_eint3, 3);
//    encoder_init(&enc3, EINT2, EINT2_P0_15, EINT_RISING_EDGE, isr_eint2, 4);
}
