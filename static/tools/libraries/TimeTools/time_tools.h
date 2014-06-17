/*
 * time_tools.h
 *
 *  Created on: 16 juin 2014
 *      Author: ludo6431
 */

#ifndef TIME_TOOLS_H_
#define TIME_TOOLS_H_

#include <sys/types.h>

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

#define TIME_CTOR() {{._v = 0, ._tr = E_TIMEREF_NONE, ._tu = E_TIMEUNIT_NONE}}
#define DATE_GETV(d) ((d)._t._v)
#define PERIOD_GETV(p) ((p)._t._v)

// basic methods definition
static inline sDate t_newDate(){
	sDate ret = TIME_CTOR();
	return ret;
}

static inline sPeriod t_newPeriod(){
	sPeriod ret = TIME_CTOR();
	return ret;
}

static inline sDate t_newDateLocalMicros(){
	sDate ret = TIME_CTOR();
	ret._t._v = micros();
	ret._t._tr = E_TIMEREF_LOCAL;
	ret._t._tu = E_TIMEUNIT_MICROS;
	return ret;
}

static inline sDate t_newDateLocalMillis(){
	sDate ret = TIME_CTOR();
	ret._t._v = millis();
	ret._t._tr = E_TIMEREF_LOCAL;
	ret._t._tu = E_TIMEUNIT_MILLIS;
	return ret;
}

extern int _t_L2GMicrosOffset;	// global = local - offset
static inline int t_setLocal2GlobalMicrosOffset(int o){ // global = local - offset
	int prev = _t_L2GMicrosOffset;
	_t_L2GMicrosOffset = o;
	return prev;
}

static inline sDate t_toGlobal(sDate d){
	sDate ret = TIME_CTOR();
	ret._t._v = d._t._v;
	ret._t._tr = E_TIMEREF_GLOBAL;
	ret._t._tu = d._t._tu;
	switch(d._t._tr){
	case E_TIMEREF_LOCAL:
		switch(d._t._tu){
		case E_TIMEUNIT_MICROS:
			ret._t._v -= _t_L2GMicrosOffset;
			break;
		case E_TIMEUNIT_MILLIS:
			ret._t._v -= _t_L2GMicrosOffset/1000;
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

static inline sDate t_toLocal(sDate d){
	sDate ret = TIME_CTOR();
	ret._t._v = d._t._v;
	ret._t._tr = E_TIMEREF_LOCAL;
	ret._t._tu = d._t._tu;
	switch(d._t._tr){
	case E_TIMEREF_GLOBAL:
		switch(d._t._tu){
		case E_TIMEUNIT_MICROS:
			ret._t._v += _t_L2GMicrosOffset;
			break;
		case E_TIMEUNIT_MILLIS:
			ret._t._v += _t_L2GMicrosOffset/1000;
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

sPeriod t_diffDates(sDate d1, sDate d2);

#endif /* TIME_TOOLS_H_ */
