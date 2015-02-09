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

#define DIR_MES_TRIGO 1
#define DIR_MES_HORAI -1


encoder_t enc1, enc2, enc3;

// Routine of interruption for encoder 1
void isr_ENC1() __attribute__ ((interrupt("IRQ")));
void isr_ENC1() {
    SCB_EXTINT = BIT(1); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    if (READ_CHA_POD1) // CHA_POD1 = 1
        enc1.dir = DIR_MES_HORAI; // sens = -1
    else
        enc1.dir = DIR_MES_TRIGO; // sens = +1

    enc1.nbticks = enc1.nbticks + enc1.dir;
}

// Routine of interruption for encoder 2
void isr_ENC2(encoder_t *e) __attribute__ ((interrupt("IRQ")));
void isr_ENC2(encoder_t *e) {
    SCB_EXTINT = BIT(0); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    if (READ_CHA_POD2) // CHA_POD2 = 1
        enc2.dir = DIR_MES_HORAI; // sens = -1
    else
        enc2.dir = DIR_MES_TRIGO; // sens = +1

    enc2.nbticks = enc2.nbticks + enc2.dir;
}

// Routine of interruption for encoder 3
void isr_ENC3(encoder_t *e) __attribute__ ((interrupt("IRQ")));
void isr_ENC3(encoder_t *e) {
    SCB_EXTINT = BIT(3); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    if (READ_CHB_POD3) // CHB_POD3 = 1
        enc3.dir = DIR_MES_HORAI; // sens = -1
    else
        enc3.dir = DIR_MES_TRIGO; // sens = +1

    enc3.nbticks = enc3.nbticks + enc3.dir;}

void encoders_init(){
    encoder_init(&enc1, EINT1, EINT1_P0_14, EINT_RISING_EDGE, isr_ENC1, 2);
    encoder_init(&enc2, EINT0, EINT0_P0_16, EINT_RISING_EDGE, isr_ENC2, 3);
    encoder_init(&enc3, EINT3, EINT3_P0_20, EINT_RISING_EDGE, isr_ENC3, 4);
}
