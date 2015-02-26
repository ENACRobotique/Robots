/*
 * motors.c
 *
 *  Created on: 10 févr. 2015
 *      Author: ludo6431
 */

#include "motors.h"

void motors_init(motor_t mots[]){
    motor_init(&mots[0], 2, 0, 4);
    motor_init(&mots[1], 4, 0, 5);
    motor_init(&mots[2], 6, 0, 6);
}
