/*
 *
 lib_synchro.h
 *
 *  Created on: 6 mars 2014
 *      Author: quentin
 */

#ifndef LIB_SYNCHRO_H_
#define LIB_SYNCHRO_H_

#include "lib_int_laser.h"

typedef struct {
    int32_t initialDelay;  // Initial delay, Delta_i
    uint32_t driftUpdatePeriod;    // 1/abs(First order drift), 1/abs(undercase delta)
    int     inc;            // signed increment to add to the offset every driftUpdatePeriod to correct drift.
}syncStruc;                 // Structure defining the synchronization parameters

typedef struct{
    uint32_t localTime;
    uint32_t turretTime;
    uint32_t period;
}syncMesStruc;

typedef struct {
    float A;    // Delta_n/Omega_n-Delta_m/Omega_m
    float B;    // 1_n/Omega_n-1_m/Omega_m
    float C;    // t_n/Omega_n-t_m/Omega_m
}ABCStruct;

enum {
    OUT_OF_SYNC,
    SYNCED
};

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


/* updateSync : Updates the correction done by millis2s and micros2s
 * /!\ Must be called often (more often than 1/syncStruct.driftFactor microsecond)
 */
void updateSync();


/* SyncComputation : Computes the synchronization parameters.
 * Arguments :
 *  t_turret : time at which the turret recorded one turn (turret reference, broadcasted by the turret, microsecond).
 *  t_laser : date at which the laser was detected (local reference, microsecond).
 *  period : Duration of last turn (measured on turret, broadcasted alongside with t_turret, microsecond).
 * Return value :
 *  OUT_OF_SYNC : no satisfactory sync has been computed
 *  SYNCED : micros2s() and millis2s() can now be used.
 *
 * Usage : feed syncComputation with data broadcasted by the turret until it returns SYNCED. After that updatesync, millis2s and micros2s can be used.
 */
int syncComputation(uint32_t t_turret, uint32_t t_laser, uint32_t period);

#endif /* LIB_SYNCHRO_H_ */
