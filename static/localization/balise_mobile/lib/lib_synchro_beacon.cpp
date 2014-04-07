/*
 * libsyncro.c
 *
 *  Created on: 6 mars 2014
 *      Author: quentin
 */

#include "lib_synchro_beacon.h"
#include "Arduino.h"
#include "../../../communication/botNet/shared/bn_debug.h"
#include "params.h"

syncStruc syncParam={0,HARDUPDATEPERIOD,HARDUPDATESIGN};    // Synchronization parameters
int32_t _offset=0;              // value to add to time to correct drift in microsecond (updated by updateSync)


// Iterative sums for least-square computation (sum_bb=sum(for i=0..N, b_i*b_i)...)
int64_t sum_OO=0,sum_D=0,sum_O=0,sum_OD=0,sum_ones=0;

sSyncPayload firstRxSyncData={0,0,-2},lastRxSyncData={0,0,-2};   // Assumption : index is INCREASED (not necessarily by 1) every time the value is updated)
syncMesStruc firstLaserMeasure={0,-2},lastLaserMeasure={0,-2};   // Assumption : index is INCREASED (not necessarily by 1) every time the value is updated)

/* micros2s : local to synchronized time (microsecond).
 * Argument :
 *  local : local date in microsecond.
 * Return value :
 *  Synchronized date (expressed in microsecond)
 */
uint32_t micros2s(uint32_t local){
    return local-_offset;
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

/* Return rank of highest bit
 */
int hbit(uint64_t val){
    int i=0;
    while(val){
        val=val>>1;
        i++;
    }
    return i;
}

/* updateSync : Updates the correction done by millis2s and micros2s
 */
void updateSync(){
    static uint32_t lastUpdate=0;
    uint32_t timeMicros=micros();

    if (!lastUpdate && !_offset && (syncParam.initialDelay || syncParam.driftUpdatePeriod)){      //only in the first call after successful synchronization
        _offset=syncParam.initialDelay + (!(syncParam.driftUpdatePeriod)?0:timeMicros/syncParam.driftUpdatePeriod)*syncParam.inc;
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
        if (lastLaserMeasure.index<0 && pload->index>=0){
            lastLaserMeasure.index=pload->index;
        }
    }

    // Check for corresponding index values, if yes computes ABCs and store it iterating sums
    if (lastLaserMeasure.index==lastRxSyncData.index && lastLaserMeasure.localTime){
        syncIntermediateCompute(lastLaserMeasure.localTime,lastRxSyncData.lastTurnDate,lastRxSyncData.period);
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
    if (!firstLaserMeasure.localTime){
        // store only value in buffer
        lastLaserMeasure.localTime=sLaser->date;
        lastLaserMeasure.index=lastRxSyncData.index;

        //duplicate it in lastLaserMeasure
        firstLaserMeasure=lastLaserMeasure;
    }
    else {
        // Computing the index : update for any missed detection, and for the last one
        while ( ((int32_t)sLaser->date-(int32_t)(lastLaserMeasure.localTime+tempIndex*lastRxSyncData.period)) > (int32_t)lastRxSyncData.period>>1){
            tempIndex++;
        }

        // store it in buffer
        lastLaserMeasure.localTime=sLaser->date;
        lastLaserMeasure.index+=tempIndex;


        // Check for corresponding index values, if yes AND useful value, computes ABCs and store it iterating sums
        if (lastLaserMeasure.index==lastRxSyncData.index){
            syncIntermediateCompute(lastLaserMeasure.localTime,lastRxSyncData.lastTurnDate,lastRxSyncData.period);
        }
    }


}

#define EVIL_SHIFT 10       // 10 : lucky value tested on only two sets of data. todo Must be tested on more data.

/* syncABCCompute : performs intermediate computations and stores the relevant informations in iterated sums.
 *
 */
void syncIntermediateCompute(uint32_t t_local, uint32_t t_turret, uint32_t period){


    int64_t Delta=t_local-t_turret;
    if ( (hbit((uint64_t)(sum_D+Delta))+hbit((uint64_t)(sum_OO+period*period)))<(63+EVIL_SHIFT) && (hbit((uint64_t)(sum_O+period))+hbit((uint64_t)(sum_OD+period*Delta)))<(63+EVIL_SHIFT) ){
#ifdef DEBUG_SYNC
        bn_printfDbg("%lu,%lu,%lu\n",t_local,t_turret,period);
#endif
        sum_D+=Delta;
        sum_O+=period;
        sum_OD+=Delta*period;
        sum_OO+=period*period;
        sum_ones+=1;
    }
#ifdef DEBUG_SYNC
    else bn_printfDbg("%lu,%lu,%lu,REJECTED(overflow)\n",t_local,t_turret,period);
#endif
}

/* SyncComputationFinal : Computes the sync parameters based on all the values recorded (least square method).
 * Usage : feed syncComputationLaser and syncComputationMsg for long enough then call syncComputationFinal.
 */
void syncComputationFinal(sSyncPayload *pload){
    int64_t det=0;

    syncComputationMsg(pload);

    det=sum_ones*sum_OO-sum_O*sum_O;

    det>>=EVIL_SHIFT;
    if (det!=0){ //todo : handle det==0
        sum_OO>>=EVIL_SHIFT;
        sum_OD>>=EVIL_SHIFT;
        syncParam.initialDelay=(sum_OO*sum_D-sum_O*sum_OD)/det;
#ifdef DEBUG_SYNC
        sum_O>>=EVIL_SHIFT;
        int64_t beta=(sum_ones*sum_OD-sum_D*sum_O)*1000/det;
        bn_printfDbg("det  %ld %ld,Delta_init %lu, beta =%ld°",(int32_t)(det>>32),(int32_t)det,syncParam.initialDelay,(int32_t)(beta*360)/1000);
#endif
    }
#undef EVIL_SHIFT

    updateSync();

}

