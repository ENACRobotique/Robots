/*
 * lib_synchro.c
 *
 *  Created on: 5 avr. 2015
 *      Author: quentin
 */

#ifndef LIB_SYNCHRO_C_
#define LIB_SYNCHRO_C_

#include "lib_synchro.h"
#include "params.h"

#ifdef ARCH_328P_ARDUINO
#include "Arduino.h"
#endif
#ifdef ARCH_X86_LINUX
#include "millis.h"
#endif
syncStruc syncParam={0,0,0};    // Synchronization parameters
int32_t _offset=0;              // value to add to time to correct drift in microsecond (updated by updateSync)

void setSyncParam(syncStruc syncParameters){
    syncParam = syncParameters;
}

syncStruc getSyncParam(){
    return syncParam;
}

/* micros2s : local to synchronized time (microsecond).
 * Argument :
 *  local : local date in microsecond.
 * Return value :
 *  Synchronized date (expressed in microsecond)
 */
uint32_t micros2s(uint32_t local){
    return local-_offset;
}

/* s2micros : synchronized to local time (microsecond).
 * Argument :
 *  local : date in microsecond.
 * Return value :
 *  local date (expressed in microsecond)
 */
uint32_t s2micros(uint32_t syncronized){
    return syncronized+_offset;
}

/* millis2s : local to synchronized time (millisecond).
 * Argument :
 *  local : local date in millisecond.
 * Return value :
 *  Synchronized date (expressed in millisecond)
 */
uint32_t millis2s(uint32_t local){
    return local-(_offset/1000);
}


/* s2millis : synchronized to local time (millisecond).
 * Argument :
 *  local : date in millisecond.
 * Return value :
 *  local date (expressed in millisecond)
 */
uint32_t s2millis(uint32_t syncronized){
    return syncronized+(_offset/1000);
}
/* updateSync : Updates the correction done by millis2s and micros2s. Must be called regularly
 */
void updateSync(){
    static uint32_t lastUpdate=0;
    uint32_t timeMicros=micros();

    if (!lastUpdate && !_offset && (syncParam.initialDelay || syncParam.driftUpdatePeriod)){      //only in the first call after successful synchronization
        _offset=syncParam.initialDelay;
        lastUpdate=timeMicros;
    }
    else if(syncParam.driftUpdatePeriod) {
        if ((timeMicros-lastUpdate)>syncParam.driftUpdatePeriod){
            _offset+=syncParam.inc;
            lastUpdate+=syncParam.driftUpdatePeriod;
        }
    }
}

#endif /* LIB_SYNCHRO_C_ */