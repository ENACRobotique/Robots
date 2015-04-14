/*
 * encoders.c
 *
 *  Created on: 9 févr. 2015
 *      Author: yoyo
 */

#include <stdlib.h>

#ifdef ARCH_LPC21XX
#include <eint.h>
#include <gpio.h>
#include <lpc214x.h>

#include "pins.h"
#endif

#include "encoders.h"

#if NB_ENCODERS != 3
#error "You can't change NB_ENCODERS without changing encoders.c as well"
#endif

#ifdef ARCH_LPC21XX
encoder_t* _encs = NULL;

// XXX on LPC2148, the pin polarity may be toggled on each interrupt if needed

// Routine of interruption for encoder 1
void isr_eint2_enc1() __attribute__ ((interrupt("IRQ")));
void isr_eint2_enc1() {
    SCB_EXTINT = BIT(2); // acknowledges interrupt
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

    _encs[2].nbticks += (gpio_read(BK_CHA_POD3, PIN_CHA_POD3) << 1) - 1;
}
#endif

void encoders_init(encoder_t encs[], motor_t mots[]) {
#ifdef ARCH_LPC21XX
    _encs = encs;

    gpio_input(BK_CHA_POD1, PIN_CHA_POD1);
    gpio_input(BK_CHA_POD2, PIN_CHA_POD2);
    gpio_input(BK_CHA_POD3, PIN_CHA_POD3);

    encoder_init(&_encs[0], &mots[0], EINT2, EINT2_P0_15, EINT_RISING_EDGE, isr_eint2_enc1, 2);
    encoder_init(&_encs[1], &mots[1], EINT0, EINT0_P0_16, EINT_RISING_EDGE, isr_eint0_enc2, 3);
    encoder_init(&_encs[2], &mots[2], EINT3, EINT3_P0_20, EINT_RISING_EDGE, isr_eint3_enc3, 4);
#elif defined(ARCH_X86_LINUX)
    encoder_init(&encs[0], &mots[0]);
    encoder_init(&encs[1], &mots[1]);
    encoder_init(&encs[2], &mots[2]);
#endif
}

/** Description:
 * Resets internal values of ticks to 0
 */
void encoders_reset(encoder_t encs[]) {
    int i;
    for (i = 0; i < NB_ENCODERS; i++) {
        encoder_update(&encs[i]);
    }
}
