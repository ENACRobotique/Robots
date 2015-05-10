/*
 * lib_syncro.h
 *
 *  Created on: 5 avr. 2015
 *      Author: quentin
 */

#ifndef LIB_SYNCHRO_H_
#define LIB_SYNCHRO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SYNC_OUT_OF_SYNC,
    SYNC_SYNCHRONIZED
};

typedef struct {
    int32_t initialDelay;       // Initial delay, Delta_i
    int32_t invDelta; // 1/abs(First order drift), 1/abs(undercase delta)
}syncStruc;


/* micros2sl : local to synchronized time (microsecond).
 * Argument :
 *  local : local date in microsecond.
 * Return value :
 *  Synchronized "laser" date (expressed in microsecond)
 */
uint32_t micros2sl(uint32_t local);

/* sl2micros : synchronized to local time (microsecond).
 * Argument :
 *  syncronized : "laser" date in microsecond.
 * Return value :
 *  local date (expressed in microsecond)
 */
uint32_t sl2micros(uint32_t syncronized);

/* millis2sl : local to synchronized time (millisecond).
 * Argument :
 *  local : local date in millisecond.
 * Return value :
 *  Synchronized "laser" date (expressed in millisecond)
 */
uint32_t millis2sl(uint32_t local);

/* sl2millis : synchronized to local time (millisecond).
 * Argument :
 *  syncronized : "laser" date in millisecond.
 * Return value :
 *  local date (expressed in millisecond)
 */
uint32_t sl2millis(uint32_t syncronized);

void setSyncParam(syncStruc syncParameters);
syncStruc getSyncParam();

#ifdef __cplusplus
    }
#endif

#endif /* LIB_SYNCHRO_H_ */
