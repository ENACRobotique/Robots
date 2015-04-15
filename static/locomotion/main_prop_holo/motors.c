/*
 * motors.c
 *
 *  Created on: 10 févr. 2015
 *      Author: ludo6431
 */

#ifdef ARCH_LPC21XX
#include "pins.h"
#endif

#include "motors.h"

void motors_init(motor_t mots[]){
#ifdef ARCH_LPC21XX
    motor_init(&mots[0], 2, BK_DIR_POD1, PIN_DIR_POD1);
    motor_init(&mots[1], 4, BK_DIR_POD2, PIN_DIR_POD2);
    motor_init(&mots[2], 6, BK_DIR_POD3, PIN_DIR_POD3);
#elif defined(ARCH_X86_LINUX)
    motor_init(&mots[0]);
    motor_init(&mots[1]);
    motor_init(&mots[2]);
#endif
}
