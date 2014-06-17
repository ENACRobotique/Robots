/*
 * time_tools.c
 *
 *  Created on: 16 juin 2014
 *      Author: ludo6431
 */

#include "time_tools.h"

int _t_Lo2GlUsOffset = 0; // global = local - offset
int _t_Lo2GlMsOffset = 0;

sPeriod tD_diff(sDate d1, sDate d2){
    sPeriod ret = TIME_CTOR();

    if(d1._t._tr == E_TIMEREF_NONE || d2._t._tr == E_TIMEREF_NONE || d1._t._tu == E_TIMEUNIT_NONE || d2._t._tu == E_TIMEUNIT_NONE){
        return ret;
    }

    // ensure both dates have the same reference
    if(d1._t._tr != d2._t._tr){
        if(d1._t._tr == E_TIMEREF_LOCAL){
            d2 = tD_conv_Lo(d2);
        }
        else{
            d1 = tD_conv_Lo(d1);
        }
    }
    if(d1._t._tr != d2._t._tr){
        return ret;
    }

    // ensure both dates have same unit
    ret._t._tr = E_TIMEREF_NONE;
    switch(d1._t._tu){
    case E_TIMEUNIT_MICROS:
        ret._t._tu = E_TIMEUNIT_MICROS;

        switch(d2._t._tu){
        case E_TIMEUNIT_MICROS:
            ret._t._v = d1._t._v - d2._t._v;
            break;
        case E_TIMEUNIT_MILLIS:
            ret._t._v = d1._t._v - (d2._t._v*1000);
            break;
        default:
            ret._t._tu = E_TIMEUNIT_NONE;
            break;
        }
        break;
    case E_TIMEUNIT_MILLIS:
        switch(d2._t._tu){
        case E_TIMEUNIT_MICROS:
            ret._t._tu = E_TIMEUNIT_MICROS;
            ret._t._v = (d1._t._v*1000) - d2._t._v;
            break;
        case E_TIMEUNIT_MILLIS:
            ret._t._tu = E_TIMEUNIT_MILLIS;
            ret._t._v = d1._t._v - d2._t._v;
            break;
        default:
            ret._t._tu = E_TIMEUNIT_NONE;
            break;
        }
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        break;
    }
    return ret;
}
