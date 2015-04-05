/*
 *
 lib_synchro.h
 *
 *  Created on: 6 mars 2014
 *      Author: quentin
 */

#ifndef LIB_SYNCHRO_BEACON_H_
#define LIB_SYNCHRO_BEACON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lib_int_laser.h"
#include "messages.h"
#include "lib_synchro.h"

typedef struct{
    uint32_t    localTime;
    int16_t     index;
}syncMesStruc; // synchronization measure structure


/* Return rank of highest bit, or -1 if val==0
 */
int hbit(uint64_t val);

/* SyncComputationMsg : Computes the synchronization parameters.
 * Usage : feed syncComputationMsg with data broadcasted by the turret until it returns SYNCED. After that updatesync, millis2s and micros2s can be used.
 *         /!\ feed also syncComputationLaser with laser data
 */
void syncComputationMsg(sSyncPayload *pload);


/* SyncComputationLaser : Computes the synchronization parameters.
 * Usage : feed syncComputationLaser with data from the elected laser buffer until sync computation is not over.
 *         /!\ feed also syncComputationMsg with data broadcasted by the turret.
 */
void syncComputationLaser(plStruct *sLaser);

/* Computes intermediate sums and store them appropriately.
 *
 */
void syncIntermediateCompute(uint32_t t_local, uint32_t t_turret, uint32_t period);

/* SyncComputationFinal : Computes the sync parameters (least square).
 * Usage : feed syncComputationLaser with data received under the flag SYNCF_END_MEASURES.
 */
void syncComputationFinal(sSyncPayload *pload);

#ifdef __cplusplus
    }
#endif

#endif /* LIB_SYNCHRO_BEACON_H_ */
