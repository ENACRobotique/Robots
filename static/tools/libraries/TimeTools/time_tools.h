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
    E_TIMEREF_NONE, // => period
    E_TIMEREF_LOCAL,
    E_TIMEREF_GLOBAL
} eTimeRef;

typedef struct __attribute__((packed)){
    uint32_t _v; // (µs), overflows after about 71.58 minutes
    eTimeRef _tr : 8;
} sDate;
typedef struct __attribute__((packed)){
    int32_t _v; // (µs), overflows after about +-35.79 minutes
} sPeriod;

// dates
#define __DC(d) ((sDate)(d))
#define TD_CTOR() {._v = 0, ._tr = E_TIMEREF_NONE}
#define TD_LoUs_CTOR(v) {._v = (v), ._tr = E_TIMEREF_LOCAL}
#define TD_LoMs_CTOR(v) {._v = (v)*1000, ._tr = E_TIMEREF_LOCAL}
#define TD_GlUs_CTOR(v) {._v = (v), ._tr = E_TIMEREF_GLOBAL}
#define TD_GlMs_CTOR(v) {._v = (v)*1000, ._tr = E_TIMEREF_GLOBAL}
#define TD_VALID(d) (__DC(d)._tr != E_TIMEREF_NONE)
#define _TD_GET_Us(d) (__DC(d)._v)
#define _TD_GET_Ms(d) (_TD_GET_Us(d)/1000)
#define TD_GET_LoUs(d) _TD_GET_Us(tD_conv_Lo(d))
#define TD_GET_LoMs(d) _TD_GET_Ms(tD_conv_Lo(d))
#define TD_GET_GlUs(d) _TD_GET_Us(tD_conv_Gl(d))
#define TD_GET_GlMs(d) _TD_GET_Ms(tD_conv_Gl(d))

// periods
#define __PC(p) ((sPeriod)(p))
#define TP_CTOR() {._v = 0}
#define TP_GET_Us(p) (__PC(p)._v)
#define TP_GET_Ms(p) (TP_GET_Us(p)/1000)
#define TD_DIFF_Us(d1, d2) TP_GET_Us(tD_diff((d1), (d2)))
#define TD_DIFF_Ms(d1, d2) TP_GET_Ms(tD_diff((d1), (d2)))

// basic methods definition
static inline sDate tD_new(){
    sDate ret = TD_CTOR();
    return ret;
}

static inline sDate tD_newFrom_LoUs(int32_t v){
    sDate ret = TD_LoUs_CTOR(v);
    return ret;
}

static inline sDate tD_newFrom_LoMs(int32_t v){
    sDate ret = TD_LoMs_CTOR(v);
    return ret;
}

static inline sDate tD_newFrom_GlUs(int32_t v){
    sDate ret = TD_GlUs_CTOR(v);
    return ret;
}

static inline sDate tD_newFrom_GlMs(int32_t v){
    sDate ret = TD_GlMs_CTOR(v);
    return ret;
}

static inline sDate tD_newNow_Lo(){
    sDate ret = TD_CTOR();
    ret._v = micros();
    ret._tr = E_TIMEREF_LOCAL;
    return ret;
}

static inline sPeriod tP_new(){
    sPeriod ret = TP_CTOR();
    return ret;
}

extern int32_t _t_Lo2GlUsOffset;	// global = local - offset

static inline uint32_t tD_setLo2GlUsOffset(int32_t o){ // global = local - offset
    int32_t prev = _t_Lo2GlUsOffset;
    _t_Lo2GlUsOffset = o;
    return prev;
}

static inline sDate tD_conv_Gl(sDate d){
    if(d._tr == E_TIMEREF_LOCAL){
        d._v -= _t_Lo2GlUsOffset;
        d._tr = E_TIMEREF_GLOBAL;
    }
    return d;
}

static inline sDate tD_conv_Lo(sDate d){
    if(d._tr == E_TIMEREF_GLOBAL){
        d._v += _t_Lo2GlUsOffset;
        d._tr = E_TIMEREF_LOCAL;
    }
    return d;
}

sPeriod tD_diff(sDate d1, sDate d2);

#endif /* TIME_TOOLS_H_ */
