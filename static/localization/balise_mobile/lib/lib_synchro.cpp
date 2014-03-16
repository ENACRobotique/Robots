/*
 * libsyncro.c
 *
 *  Created on: 6 mars 2014
 *      Author: quentin
 */

#include "lib_synchro.h"
#include "Arduino.h"

syncStruc syncParam={0,0,0};
int32_t _offset;            // value to add to time to correct drift in microsecond (updated by updateSync)

#if 0
syncMesStruc *firstMesure=NULL;
ABCStruct *firstABC=NULL;
syncStruc **syncStrucBuffer=NULL;
#endif



/* micros2s : local to synchronized time (microsecond).
 * Argument :
 *  local : local date in microsecond.
 * Return value :
 *  Synchronized date (expressed in microsecond)
 */
uint32_t micros2s(uint32_t local){
    updateSync();
    return local+_offset;
}

/* millis2s : local to synchronized time (millisecond).
 * Argument :
 *  local : local date in millisecond.
 * Return value :
 *  Synchronized date (expressed in millisecond)
 */
uint32_t millis2s(uint32_t local){
    updateSync();
    return local+(_offset/1000);
}

/* updateSync : Updates the correction done by millis2s and micros2s
 */
void updateSync(){
    static uint32_t lastUpdate=0;
    uint32_t timeMicros=micros();

    if (!lastUpdate && !_offset && (syncParam.initialDelay || syncParam.driftUpdatePeriod)){      //only in the first call after successful synchronization
        _offset=syncParam.initialDelay + (syncParam.driftUpdatePeriod?0:timeMicros/syncParam.driftUpdatePeriod);
        lastUpdate=timeMicros-(syncParam.driftUpdatePeriod?0:timeMicros%syncParam.driftUpdatePeriod);
    }
    else if(syncParam.driftUpdatePeriod) {
        if ((timeMicros-lastUpdate)>syncParam.driftUpdatePeriod){
            _offset+=syncParam.inc;
            lastUpdate+=syncParam.driftUpdatePeriod;
        }
    }
}

/* SyncComputationMsg : Computes the synchronization parameters.
 * Usage : feed syncComputationMsg with data broadcasted by the turret, including the first message stating "begin measure (i.e. index=0)" until it returns SYNCED. After that updatesync, millis2s and micros2s can be used.
 *         /!\ feed also syncComputationLaser with laser data
 */
int syncComputationMsg(sSyncPayload *pload){

    // if first received, initialize

    // store it in rotating buffer

    // Check for corresponding index values, and store them if useful

    return SYNC_OUT_OF_SYNC;
}


/* SyncComputationLaser : Computes the synchronization parameters.
 * Usage : feed syncComputationLaser with data from the elected laser buffer until sync computation is not over.
 *         /!\ feed also syncComputationMsg with data broadcasted by the turret.
 */
void syncComputationLaser(plStruct *sLaser){
    static int syncLocalIndex=-2;

    // Computing the index

    // store it in rotating buffer

    // Check for corresponding index values, and store them if useful

}




#if 0
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
int syncComputation(uint32_t t_turret, uint32_t t_laser, uint32_t period){
    ABCStruct tempABC;
    float littleDelta;

    static int iBuf=0;

    // If not already recorded, stores the first argument triplets
    if (firstMesure==NULL){
        firstMesure=(syncMesStruc*)malloc(sizeof(syncMesStruc));
        firstMesure->localTime=t_laser;
        firstMesure->period=period;
        firstMesure->turretTime=t_turret;
        return OUT_OF_SYNC;
    }
    else {
        uint32_t Delta_n=firstMesure->turretTime-firstMesure->localTime;
        uint32_t Delta_m=t_turret-t_laser;

        // Computes ABC
        tempABC.A=(float)Delta_n/firstMesure->period-(float)Delta_m/period;
        tempABC.B=(float)1./firstMesure->period-(float)1./period;
        tempABC.C=(float)firstMesure->turretTime/firstMesure->period-(float)t_laser/period;

        // if first ABC, store it Specially (and reserve memory for the rotating buffer)
        if (firstABC==NULL){
            firstABC=(ABCStruct*)malloc(sizeof(ABCStruct));
            *firstABC=tempABC;
            syncStrucBuffer=(syncStruc**)malloc(sizeof(syncStruc)*SYNC_STRUCT_BUFF_SIZE);
        }
        // otherwise,
        else {
            // Computes little delta && Delta_i && store it
            littleDelta = ( tempABC.A - firstABC->A*firstABC->B/tempABC.B ) / (tempABC.C-tempABC.B*firstABC->C/firstABC->B);
            syncStrucBuffer[iBuf]->initialDelay = (firstABC->A - littleDelta*firstABC->C)/firstABC->B;
            if (littleDelta!=0){
                syncStrucBuffer[iBuf]->driftUpdatePeriod=1./fabs(littleDelta);
                syncStrucBuffer[iBuf]->inc= littleDelta<0 ? -1 : 1;
            }
            else {
                syncStrucBuffer[iBuf]->driftUpdatePeriod=0;
                syncStrucBuffer[iBuf]->inc=0;
            }

            iBuf=(iBuf+1)%SYNC_STRUCT_BUFF_SIZE;

            // if we have old enough values, compute the mean and store it as a final result. fixme : what if we don't have enough values ?
            if ((t_laser-firstMesure->localTime)>SYNC_TIME_DURATION){
                int32_t meanUpdatePeriod=0;
                int32_t meanInitialDelay=0;

                for (int j=0; j<SYNC_STRUCT_BUFF_SIZE && (syncStrucBuffer[j]->inc || syncStrucBuffer[iBuf]->driftUpdatePeriod) ; j++){
                    meanUpdatePeriod+=syncStrucBuffer[j]->driftUpdatePeriod*syncStrucBuffer[j]->inc;
                    meanInitialDelay+=syncStrucBuffer[j]->initialDelay;
                }

                syncParam.driftUpdatePeriod=abs(meanUpdatePeriod);
                syncParam.inc= meanUpdatePeriod<0 ? -1 : 1;
                syncParam.initialDelay=meanInitialDelay;

                free(firstMesure);
                free(firstABC);
                free(syncStrucBuffer);
            }
        }

    }


    return OUT_OF_SYNC;
}

#endif
