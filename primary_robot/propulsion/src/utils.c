/*
 * utils.c
 *
 *  Created on: 3 mai 2017
 *      Author: fabien
 */
#include "utils.h"
#include "Arduino.h"

double constrainAngle(double x){
    x = fmod(x + PI,TWO_PI);
    if (x < 0)
        x += TWO_PI;
    return x - PI;
}



