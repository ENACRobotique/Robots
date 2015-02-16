/*
 * encoders.c
 *
 *  Created on: 9 févr. 2015
 *      Author: yoyo
 */

#include <eint.h>
#include <gpio.h>
#include <lpc214x.h>
#include <param.h>
#include "encoders.h"

encoder_t* _encs;

#if NB_ENCODERS != 3
#error "You can't change NB_ENCODERS without changing encoders.c as well"
#endif

// Routine of interruption for encoder 1
void isr_eint1_enc1() __attribute__ ((interrupt("IRQ")));
void isr_eint1_enc1() {
    SCB_EXTINT = BIT(1); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    _encs[0].nbticks += (gpio_read(BK_CHA_POD1, PIN_CHA_POD1) << 1) - 1;
}

// Routine of interruption for encoder 2
void isr_eint0_enc2() __attribute__ ((interrupt("IRQ")));
void isr_eint0_enc2() {
    SCB_EXTINT = BIT(0); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    _encs[1].nbticks += (gpio_read(BK_CHA_POD2, PIN_CHA_POD2) << 1) - 1;
}

// Routine of interruption for encoder 3
void isr_eint3_enc3() __attribute__ ((interrupt("IRQ")));
void isr_eint3_enc3() {
    SCB_EXTINT = BIT(3); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    _encs[2].nbticks += (gpio_read(BK_CHB_POD3, PIN_CHB_POD3) << 1) - 1;
}

void encoders_init(encoder_t encs[]) {
    _encs = encs;

    encoder_init(&_encs[0], EINT1, EINT1_P0_14, EINT_RISING_EDGE, isr_eint1_enc1, 2);
    encoder_init(&_encs[1], EINT0, EINT0_P0_16, EINT_RISING_EDGE, isr_eint0_enc2, 3);
    encoder_init(&_encs[2], EINT3, EINT3_P0_20, EINT_RISING_EDGE, isr_eint3_enc3, 4);
}
