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
    int32_t initialDelay;  // Initial delay, Delta_i
    uint32_t driftUpdatePeriod;    // 1/abs(First order drift), 1/abs(undercase delta)
    int     inc;            // signed increment to add to the offset every driftUpdatePeriod to correct drift.
}syncStruc;                 // Structure defining the synchronization parameters

/* updateSync : Updates the correction done by millis2s and micros2s
 * /!\ Must be called often (more often than 1/syncStruct.driftFactor microsecond)
 */
void updateSync();

/* micros2s : local to synchronized time (microsecond).
 * Argument :
 *  local : local date in microsecond.
 * Return value :
 *  Synchronized date (expressed in microsecond)
 */
uint32_t micros2s(uint32_t local);

/* millis2s : local to synchronized time (millisecond).
 * Argument :
 *  local : local date in millisecond.
 * Return value :
 *  Synchronized date (expressed in millisecond)
 */
uint32_t millis2s(uint32_t local);

void setSyncParam(syncStruc syncParameters);
syncStruc getSyncParam();

#ifdef __cplusplus
    }
#endif

#endif /* LIB_SYNCHRO_H_ */
