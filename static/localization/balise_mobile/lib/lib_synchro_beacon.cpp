/*
 * libsyncro.c
 *
 *  Created on: 6 mars 2014
 *      Author: quentin
 */

#include "lib_synchro_beacon.h"
#include "Arduino.h"

syncStruc syncParam={0,0,0};
int32_t _offset;            // value to add to time to correct drift in microsecond (updated by updateSync)

#define BUF_ABC_SIZE 10

ABCStruct firstABC;
ABCStruct bufABC[BUF_ABC_SIZE];
int bufABCinsert=0,bufABCsize=0;

sSyncPayload firstRxSyncData={0,0,-2},lastRxSyncData={0,0,-2};   // Assumption : index is INCREASED (not necessarily by 1) every time the value is updated)
syncMesStruc firstLaserMeasure={0,-2},lastLaserMeasure={0,-2};   // Assumption : index is INCREASED (not necessarily by 1) every time the value is updated)


int syncLocalIndex=-2;


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
void syncComputationMsg(sSyncPayload *pload){

    // if no update, return
    if (pload->lastTurnDate==lastRxSyncData.lastTurnDate) return;

    // store it in buffer
    lastRxSyncData=*pload;

    //if it is the very first value, store it in special variable, upload laser index and return
    if (!firstRxSyncData.period){
        firstRxSyncData=lastRxSyncData;
        // if first received, initialize
        if (syncLocalIndex<0 && pload->index>=0){
            syncLocalIndex=pload->index;
        }
    }

    // Check for corresponding index values, if yes computes ABCs and store it in rotating buffer
    if (lastLaserMeasure.index==lastRxSyncData.index){
        syncABCCompute(lastLaserMeasure.localTime,lastRxSyncData.lastTurnDate,lastRxSyncData.period);
    }

}


/* SyncComputationLaser : Computes the synchronization parameters.
 * Usage : feed syncComputationLaser with data from the elected laser buffer while sync computation is not over.
 *         /!\ feed also syncComputationMsg with data broadcasted by the turret.
 */
void syncComputationLaser(plStruct *sLaser){
    int16_t tempIndex=0;    //increment to add to lastLaserMeasure

    // if no update, return
    if (sLaser->date==lastLaserMeasure.localTime) return;

    // if first value, do not update the index, but store value
    if (!sLaser->date){
        // store only value in buffer
        lastLaserMeasure.localTime=sLaser->date;

        //duplicate it in lastLaserMeasure
        firstLaserMeasure=lastLaserMeasure;
    }
    else {
        // Computing the index : update for any missed detection, and for the last one
        while ( (sLaser->date-lastLaserMeasure.localTime+tempIndex*lastRxSyncData.period) < lastRxSyncData.period>>1){
            tempIndex++;
        }

        // store it in buffer
        lastLaserMeasure.localTime=sLaser->date;
        lastLaserMeasure.index+=tempIndex;

        // Check for corresponding index values, if yes AND useful value, computes ABCs and store it in rotating buffer
        if (lastLaserMeasure.index==lastRxSyncData.index){
            syncABCCompute(lastLaserMeasure.localTime,lastRxSyncData.lastTurnDate,lastRxSyncData.period);
        }
    }


}


void syncABCCompute(uint32_t t_local, uint32_t t_turret, uint32_t period){

    // avoid division by 0
    if (firstRxSyncData.period && period){
        ABCStruct tempABC;
        uint32_t Delta_n=firstRxSyncData.lastTurnDate -firstLaserMeasure.localTime;
        uint32_t Delta_m=t_turret-t_local;

        // Computes ABC
        tempABC.A=(float)Delta_n/firstRxSyncData.period-(float)Delta_m/period;
        tempABC.B=(float)1./firstRxSyncData.period-(float)1./period;
        tempABC.C=(float)firstRxSyncData.lastTurnDate/firstRxSyncData.period-(float)t_local/period;

        // if first value computed, store it specially
        if (!bufABCsize) firstABC=tempABC;

        //stores it and updates insert index and size
        bufABC[bufABCinsert]=tempABC;
        bufABCinsert=(bufABCinsert+1)%BUF_ABC_SIZE;
        if (bufABCsize<BUF_ABC_SIZE) bufABCsize++;
    }
}

/* SyncComputationFinal : Computes the sync parameters (least square).
 * Usage : feed syncComputationLaser with data received under the flag SYNCF_END_MEASURES.
 */
void syncComputationFinal(sSyncPayload *pload){

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
