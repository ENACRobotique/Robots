/*
 * test_tools.c
 *
 *  Created on: 9 oct. 2014
 *      Author: ludo6431
 */

#include <stdlib.h>

#include "test_tools.h"

float rand_uniform(float min, float max) {
    return ((double) rand() / ((double) RAND_MAX + 1)) * (max - min) + min;
}
