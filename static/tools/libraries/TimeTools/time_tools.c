/*
 * time_tools.c
 *
 *  Created on: 16 juin 2014
 *      Author: ludo6431
 */

#include "time_tools.h"

int32_t _t_Lo2GlUsOffset = 0; // global = local - offset

sPeriod tD_diff(sDate d1, sDate d2){
    sPeriod ret = TP_CTOR();

    if(!TD_VALID(d1) || !TD_VALID(d2)){
        return ret;
    }

    // ensure both dates have the same reference
    if(d1._tr != d2._tr){
        d1 = tD_conv_Lo(d1);
        d2 = tD_conv_Lo(d2);
    }

    ret._v = d1._v - d2._v;

    return ret;
}
