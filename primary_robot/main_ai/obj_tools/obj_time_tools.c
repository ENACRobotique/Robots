/*
 * obj_time_tools.c
 *
 *  Created on: 18 avr. 2014
 *      Author: ludo6431
 */

#include <millis.h>
#include <stdint.h>
#include <sys/types.h>
#include "obj_time_tools.h"

// overflow proof t1 - t2
int32_t time_diff(uint32_t t1, uint32_t t2){
    uint32_t time = micros() + 100000000UL; // FIXME, use synchronized micros() (to be sure <time> is more recent than <t1> and <t2>)
    uint32_t dt1, dt2;

    dt1 = time - t1;
    dt2 = time - t2;

    return dt1 > dt2 ? -(int32_t)(dt1 - dt2) : (int32_t)(dt2 - dt1);
}
