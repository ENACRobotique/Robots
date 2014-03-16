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
#include "messages.h"


#define SYNC_STRUCT_BUFF_SIZE       16          // Number of points to compute the average
#define SYNC_LASER_ELECTION_TIME    2000000     // Time during which we will measure the best laser interruption to measure the clock drift (µs)
#define SYNC_TIME_DURATION          10000000    // Duration of synchronization

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
    SYNC_BEGIN_ELECTION,
    SYNC_BEGIN_MEASURES,
    SYNC_OUT_OF_SYNC,
    SYNC_SYNCHRONIZED
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

/* SyncComputationMsg : Computes the synchronization parameters.
 * Usage : feed syncComputationMsg with data broadcasted by the turret until it returns SYNCED. After that updatesync, millis2s and micros2s can be used.
 *         /!\ feed also syncComputationLaser with laser data
 */
int syncComputationMsg(sSyncPayload *pload);


/* SyncComputationLaser : Computes the synchronization parameters.
 * Usage : feed syncComputationLaser with data from the elected laser buffer until sync computation is not over.
 *         /!\ feed also syncComputationMsg with data broadcasted by the turret.
 */
void syncComputationLaser(plStruct *sLaser);


#endif /* LIB_SYNCHRO_H_ */
