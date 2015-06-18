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
syncStruc syncParam={0,0};    // Synchronization parameters

void setSyncParam(syncStruc syncParameters){
    syncParam = syncParameters;
}

syncStruc getSyncParam(){
    return syncParam;
}

/* micros2sl : local to synchronized time (microsecond).
 * Argument :
 *  local : local date in microsecond.
 * Return value :
 *  Synchronized "laser" date (expressed in microsecond)
 */
uint32_t micros2sl(uint32_t local){
    return local + local/syncParam.invDelta + syncParam.initialDelay;
}

/* sl2micros : synchronized to local time (microsecond).
 * Argument :
 *  syncronized : "laser" date in microsecond.
 * Return value :
 *  local date (expressed in microsecond)
 */
uint32_t sl2micros(uint32_t syncronized){
    int64_t ret;
    if (syncParam.invDelta) {
        ret = (syncronized - syncParam.initialDelay)*syncParam.invDelta;
        ret /= (1+syncParam.invDelta);
    }
    else ret = syncronized - syncParam.initialDelay;
    return ret;
}

/* millis2sl : local to synchronized time (millisecond).
 * Argument :
 *  local : local date in millisecond.
 * Return value :
 *  Synchronized "laser" date (expressed in millisecond)
 */
uint32_t millis2sl(uint32_t local){
    return local + local/(syncParam.invDelta/1000) + syncParam.initialDelay/1000;
}


/* sl2millis : synchronized to local time (millisecond).
 * Argument :
 *  syncronized : "laser" date in millisecond.
 * Return value :
 *  local date (expressed in millisecond)
 */
uint32_t sl2millis(uint32_t syncronized){
    int64_t ret;
    if (syncParam.invDelta) {
        ret= (syncronized - syncParam.initialDelay/1000)*syncParam.invDelta/1000;
        ret /= (1+syncParam.invDelta/1000);
    }
    else ret = syncronized - syncParam.initialDelay;
    return ret;
}

#endif /* LIB_SYNCHRO_C_ */
