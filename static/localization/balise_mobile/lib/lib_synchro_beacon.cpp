/*
 * libsyncro.c
 *
 *  Created on: 6 mars 2014
 *      Author: quentin
 */

#include "lib_synchro_beacon.h"
#include "Arduino.h"

syncStruc syncParam={0,0,0};    // Synchronization parameters
int32_t _offset=0;              // value to add to time to correct drift in microsecond (updated by updateSync)


// Iterative sums for least-square computation (sum_bb=sum(for i=0..N, b_i*b_i)...)
float sum_ab=0,sum_ac=0,sum_bb=0,sum_bc=0,sum_cc=0;


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
    return local-_offset;
}

/* millis2s : local to synchronized time (millisecond).
 * Argument :
 *  local : local date in millisecond.
 * Return value :
 *  Synchronized date (expressed in millisecond)
 */
uint32_t millis2s(uint32_t local){
    updateSync();
    return local-(_offset/1000);
}

/* updateSync : Updates the correction done by millis2s and micros2s
 */
void updateSync(){
    static uint32_t lastUpdate=0;
    uint32_t timeMicros=micros();

    if (!lastUpdate && !_offset && (syncParam.initialDelay || syncParam.driftUpdatePeriod)){      //only in the first call after successful synchronization
        _offset=syncParam.initialDelay + (syncParam.driftUpdatePeriod?0:timeMicros/syncParam.driftUpdatePeriod)*syncParam.inc;
        lastUpdate=timeMicros-(syncParam.driftUpdatePeriod?0:timeMicros%syncParam.driftUpdatePeriod);
    }
    else if(syncParam.driftUpdatePeriod) {
        if ((timeMicros-lastUpdate)>syncParam.driftUpdatePeriod){
            _offset+=syncParam.inc;
            lastUpdate+=syncParam.driftUpdatePeriod;
        }
    }
}

/* SyncComputationMsg : Stores last received values and eventually computes the synchronization parameters.
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

    // Check for corresponding index values, if yes computes ABCs and store it iterating sums
    if (lastLaserMeasure.index==lastRxSyncData.index){
        syncABCCompute(lastLaserMeasure.localTime,lastRxSyncData.lastTurnDate,lastRxSyncData.period);
    }

}


/* SyncComputationLaser : stores last laser value and eventually computes the synchronization parameters.
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

        // Check for corresponding index values, if yes AND useful value, computes ABCs and store it iterating sums
        if (lastLaserMeasure.index==lastRxSyncData.index){
            syncABCCompute(lastLaserMeasure.localTime,lastRxSyncData.lastTurnDate,lastRxSyncData.period);
        }
    }


}

/* syncABCCompute : computes the A, B, and C terms and stores the relevant informations in iterated sums.
 *
 */
void syncABCCompute(uint32_t t_local, uint32_t t_turret, uint32_t period){

    // avoid division by 0
    if (firstRxSyncData.period && period){
        uint32_t Delta_n=firstLaserMeasure.localTime-firstRxSyncData.lastTurnDate;
        uint32_t Delta_m=t_local-t_turret;

        // Computes ABC
        float A=(float)Delta_n/firstRxSyncData.period-(float)Delta_m/period;
        float B=(float)1./firstRxSyncData.period-(float)1./period;
        float C=(float)firstLaserMeasure.localTime/firstRxSyncData.period-(float)t_local/period;

        // compute & store the interesting values
        sum_ab+=A*B;
        sum_ac+=A*C;
        sum_bb+=B*B;
        sum_bc+=B*C;
        sum_cc+=C*C;
    }
}

/* SyncComputationFinal : Computes the sync parameters based on all the values recorded (least square method).
 * Usage : feed syncComputationLaser and syncComputationMsg for long enough then call syncComputationFinal.
 */
void syncComputationFinal(sSyncPayload *pload){
    float det;
    float delta;

    syncComputationMsg(pload);

    det=sum_bb*sum_cc-sum_bc*sum_bc;
    if (det!=0){ //todo : handle det==0
        syncParam.initialDelay=(sum_ab*sum_cc-sum_bc*sum_ac)/det;
        delta=(-sum_bc*sum_bc+sum_ac*sum_bb)/det;
        if (delta!=0) syncParam.driftUpdatePeriod=fabs(1./delta);
        syncParam.inc=(delta>0?1:-1);
    }
}

