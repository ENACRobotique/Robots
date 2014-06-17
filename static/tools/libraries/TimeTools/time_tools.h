/*
 * time_tools.h
 *
 *  Created on: 16 juin 2014
 *      Author: ludo6431
 */

#ifndef TIME_TOOLS_H_
#define TIME_TOOLS_H_

#include <stdint.h>

#ifdef ARCH_328P_ARDUINO
#include "Arduino.h"
#elif defined(ARCH_X86_LINUX)
#include "../../../core/linux/libraries/Millis/millis.h"
#elif defined(ARCH_LPC21XX)
#include <sys_time.h>
#elif defined(ARCH_LM4FXX)
#include "time.h"
#else
#error "no ARCH_XXX symbol defined"
#endif

// basic types definition
typedef enum{
    E_TIMEUNIT_NONE,
    E_TIMEUNIT_MICROS, // overflows after about 71.58 minutes
    E_TIMEUNIT_MILLIS // overflows after about 1193 days
} eTimeUnit;

typedef enum{
    E_TIMEREF_NONE,
    E_TIMEREF_LOCAL,
    E_TIMEREF_GLOBAL
} eTimeRef;

typedef struct __attribute__((packed)){
    int32_t _v;
    eTimeRef _tr :4;
    eTimeUnit _tu :4;
} __sTime;
typedef struct __attribute__((packed)){
    __sTime _t;
} sDate;
typedef struct __attribute__((packed)){
    __sTime _t;
} sPeriod;

#define _DC(d) ((sDate)(d))
#define _PC(p) ((sPeriod)(p))
#define TIME_CTOR() {{._v = 0, ._tr = E_TIMEREF_NONE, ._tu = E_TIMEUNIT_NONE}}
#define TD_LoUs_CTOR(v) {{._v = (int32_t)(v), ._tr = E_TIMEREF_LOCAL, ._tu = E_TIMEUNIT_MICROS}}
#define TD_LoMs_CTOR(v) {{._v = (int32_t)(v), ._tr = E_TIMEREF_LOCAL, ._tu = E_TIMEUNIT_MILLIS}}
#define TD_GlUs_CTOR(v) {{._v = (int32_t)(v), ._tr = E_TIMEREF_GLOBAL, ._tu = E_TIMEUNIT_MICROS}}
#define TD_GlMs_CTOR(v) {{._v = (int32_t)(v), ._tr = E_TIMEREF_GLOBAL, ._tu = E_TIMEUNIT_MILLIS}}
#define TD_VALID(d) (_DC(d)._t._tr != E_TIMEREF_NONE && _DC(d)._t._tu != E_TIMEUNIT_NONE)
#define TD_GET(d) ((uint32_t)(_DC(d)._t._v))
#define TD_GET_LoUs(d) TD_GET(tD_conv_LoUs(d))
#define TD_GET_LoMs(d) TD_GET(tD_conv_LoMs(d))
#define TD_GET_GlUs(d) TD_GET(tD_conv_GlUs(d))
#define TD_GET_GlMs(d) TD_GET(tD_conv_GlMs(d))
#define TP_GET(p) (_PC(p)._t._v)
#define TD_GETDIFF(d1, d2) TP_GET(tD_diff((d1), (d2)))

// basic methods definition
static inline sDate tD_new(){
    sDate ret = TIME_CTOR();
    return ret;
}

static inline sDate tD_newFrom_LoUs(int v){
    sDate ret = TD_LoUs_CTOR(v);
    return ret;
}

static inline sDate tD_newFrom_LoMs(int v){
    sDate ret = TD_LoMs_CTOR(v);
    return ret;
}

static inline sDate tD_newFrom_GlUs(int v){
    sDate ret = TD_GlUs_CTOR(v);
    return ret;
}

static inline sDate tD_newFrom_GlMs(int v){
    sDate ret = TD_GlMs_CTOR(v);
    return ret;
}

static inline sPeriod tP_new(){
    sPeriod ret = TIME_CTOR();
    return ret;
}

static inline sDate tD_newNow_LoUs(){
    sDate ret = TIME_CTOR();
    ret._t._v = micros();
    ret._t._tr = E_TIMEREF_LOCAL;
    ret._t._tu = E_TIMEUNIT_MICROS;
    return ret;
}

static inline sDate tD_newNow_LoMs(){
    sDate ret = TIME_CTOR();
    ret._t._v = millis();
    ret._t._tr = E_TIMEREF_LOCAL;
    ret._t._tu = E_TIMEUNIT_MILLIS;
    return ret;
}

extern int _t_Lo2GlUsOffset;	// global = local - offset
extern int _t_Lo2GlMsOffset;    // global = local - offset

static inline int tD_setLo2GlUsOffset(int o){ // global = local - offset
    int prev = _t_Lo2GlUsOffset;
    _t_Lo2GlUsOffset = o;
    _t_Lo2GlMsOffset = o/1000;
    return prev;
}

static inline int tD_setLo2GlMsOffset(int o){ // global = local - offset
    int prev = _t_Lo2GlMsOffset;
    _t_Lo2GlUsOffset = o*1000;
    _t_Lo2GlMsOffset = o;
    return prev;
}

static inline sDate tD_conv_Gl(sDate d){
    sDate ret = TIME_CTOR();
    ret._t._v = d._t._v;
    ret._t._tr = E_TIMEREF_GLOBAL;
    ret._t._tu = d._t._tu;
    switch(d._t._tr){
    case E_TIMEREF_LOCAL:
        switch(d._t._tu){
        case E_TIMEUNIT_MICROS:
            ret._t._v -= _t_Lo2GlUsOffset;
            break;
        case E_TIMEUNIT_MILLIS:
            ret._t._v -= _t_Lo2GlMsOffset;
            break;
        default:
            ret._t._tu = E_TIMEUNIT_NONE;
            break;
        }
        break;
    case E_TIMEREF_GLOBAL:
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        break;
    }
    return ret;
}

static inline sDate tD_conv_GlUs(sDate d){
    sDate ret = TD_GlUs_CTOR(d._t._v);

    switch(d._t._tu){
    case E_TIMEUNIT_MICROS:
        break;
    case E_TIMEUNIT_MILLIS:
        ret._t._v *= 1000;
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        return ret;
    }

    switch(d._t._tr){
    case E_TIMEREF_LOCAL:
        ret._t._v -= _t_Lo2GlUsOffset;
        break;
    case E_TIMEREF_GLOBAL:
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        return ret;
    }

    return ret;
}

static inline sDate tD_conv_GlMs(sDate d){
    sDate ret = TD_GlMs_CTOR(d._t._v);

    switch(d._t._tu){
    case E_TIMEUNIT_MICROS:
        ret._t._v /= 1000;
        break;
    case E_TIMEUNIT_MILLIS:
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        return ret;
    }

    switch(d._t._tr){
    case E_TIMEREF_LOCAL:
        ret._t._v -= _t_Lo2GlMsOffset;
        break;
    case E_TIMEREF_GLOBAL:
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        return ret;
    }

    return ret;
}

static inline sDate tD_conv_Lo(sDate d){
    sDate ret = TIME_CTOR();
    ret._t._v = d._t._v;
    ret._t._tr = E_TIMEREF_LOCAL;
    ret._t._tu = d._t._tu;
    switch(d._t._tr){
    case E_TIMEREF_GLOBAL:
        switch(d._t._tu){
        case E_TIMEUNIT_MICROS:
            ret._t._v += _t_Lo2GlUsOffset;
            break;
        case E_TIMEUNIT_MILLIS:
            ret._t._v += _t_Lo2GlMsOffset;
            break;
        default:
            ret._t._tu = E_TIMEUNIT_NONE;
            break;
        }
        break;
    case E_TIMEREF_LOCAL:
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        break;
    }
    return ret;
}

static inline sDate tD_conv_LoUs(sDate d){
    sDate ret = TD_LoUs_CTOR(d._t._v);

    switch(d._t._tu){
    case E_TIMEUNIT_MICROS:
        break;
    case E_TIMEUNIT_MILLIS:
        ret._t._v *= 1000;
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        return ret;
    }

    switch(d._t._tr){
    case E_TIMEREF_LOCAL:
        break;
    case E_TIMEREF_GLOBAL:
        ret._t._v += _t_Lo2GlUsOffset;
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        return ret;
    }

    return ret;
}

static inline sDate tD_conv_LoMs(sDate d){
    sDate ret = TD_LoMs_CTOR(d._t._v);

    switch(d._t._tu){
    case E_TIMEUNIT_MICROS:
        ret._t._v /= 1000;
        break;
    case E_TIMEUNIT_MILLIS:
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        return ret;
    }

    switch(d._t._tr){
    case E_TIMEREF_LOCAL:
        break;
    case E_TIMEREF_GLOBAL:
        ret._t._v += _t_Lo2GlMsOffset;
        break;
    default:
        ret._t._tu = E_TIMEUNIT_NONE;
        return ret;
    }

    return ret;
}

sPeriod tD_diff(sDate d1, sDate d2);

#endif /* TIME_TOOLS_H_ */
