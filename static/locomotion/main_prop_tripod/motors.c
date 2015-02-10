/*
 * motors.c
 *
 *  Created on: 10 févr. 2015
 *      Author: ludo6431
 */

#include <motor.h>
#include "motors.h"

motor_t mot1, mot2, mot3;

void motors_init(){
    motor_init(&mot1, 2, 0, 4);
    motor_init(&mot2, 4, 0, 5);
    motor_init(&mot3, 6, 0, 6);
}
