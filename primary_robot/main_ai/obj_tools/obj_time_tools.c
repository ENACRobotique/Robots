/*
 * obj_time_tools.c
 *
 *  Created on: 18 avr. 2014
 *      Author: ludo6431
 */


#include <stdint.h>
#include <sys/types.h>
#include "obj_time_tools.h"

int32_t time_diff(uint32_t t1, uint32_t t2){
    return (int32_t)(t1 - t2); // FIXME, works only if t1 more recent than t2
}
