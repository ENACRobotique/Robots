/*
 * lib_synchro_wire.c
 *
 *  Created on: 5 avr. 2015
 *      Author: quentin
 */

#include "timeout.h"
#include "lib_synchro_wire.h"

#if defined(WIREDSYNC_BENCHMARK) && defined(ARCH_X86_LINUX)
#include <stdio.h>
#endif

#define SUM_SHIFT (0) // how much will the values be shifted before entering the sums

#ifndef abs
#define abs(x) ((x)>0?(x):-(x)
#endif

// static variables to store values between different calls
static wsType_t sum_ones=0;    // sum of one
static wsType_t sum_lt=0;      // sum of local dates
static wsType_t sum_ltsq=0;     // sum of (local dates)^2
static wsType_t sum_gt_lt=0;   // sum of (global date - local date)
static wsType_t sum_lt_gt_lt=0;// sum of (local date)*(global date - local date)
#ifndef WIREDSYNC_BENCHMARK
/* wiredSync_waitSignal : function that must be in the main loop, and wiats for the wired synchronization signal.
 * "SyncParam" is reset in EVERY TIME this signal is received.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  None
 * Returned value :
 *  SYNC_OUT_OF_SYNC while no synchronizing signal has been received
 *  SYNC_SYNCHRONIZED after the synchronizing signal has been received
 */
int wiredSync_waitSignal(){
    static int synchronized = 0;
    syncStruc tmpStruc = getSyncParam();
    // record current time
    uint32_t begin = micros();
    // if signal is here
    while (wiredSync_signalPresent()==WIREDSYNC_SIGNALISHERE);
    // if signal stayed here for more than debounce time, update initial time delay.
    uint32_t end = micros();
    if ((end-begin)/1000 > WIREDSYNC_DEBOUNCE){
        tmpStruc.initialDelay = end - WIREDSYNC_INITIAL;
        setSyncParam(tmpStruc);
        updateSync();
        synchronized = 1;
    }
    return (synchronized?SYNC_SYNCHRONIZED:SYNC_OUT_OF_SYNC);
}
#endif
/* wiredSync_intermediateCompute :  records a new set of measures for the synchronization.
 * The actual computation is done in wiredSync_finalCompute.
 * This function is to be called on the device which has NOT the reference clock.
 * Arguments :
 *  gTime : global date of the event, i.e. time of the reference clock when the synchronizing event occurred
 *  lTime : local date of the same event (this one is pretty self-explaining)
 * Return value :
 *  none
 */
void wiredSync_intermediateCompute(wsType_t gTime, wsType_t lTime){
    sum_ones += 1;
    sum_lt += lTime;
    sum_ltsq += lTime*lTime;
    sum_gt_lt += gTime - lTime;
    sum_lt_gt_lt += lTime*(gTime - lTime);
}

/* wiredSync_finalCompute : actual computation of the synchronization.
 * This function is to be called on the device which has NOT the reference clock.
 * Arguments :
 *  reset : if reset != 0, will reset the sums (i.e. delete previous measures)
 * Returned value :
 *  SYNC_OUT_OF_SYNC while no synchronization has been successful (syncParam is not modified)
 *  SYNC_SYNCHRONIZED after the synchronization is achieved, and syncParams updated
 */
int wiredSync_finalCompute(int reset){
    wsType_t inv_delta=0;    // inverse of first order difference (linear drift)
    wsType_t inv_delta_den=0;// denominator of the inverse of delta
    wsType_t offset=0;       // initial offset
    wsType_t offset_num=0;
    wsType_t det = 0;
    int retVal;

    det= sum_ltsq * sum_ones - sum_lt*sum_lt;

    if (det!=0){
        inv_delta_den = sum_ones*sum_lt_gt_lt - sum_lt*sum_gt_lt;
        inv_delta = det / inv_delta_den;
        offset_num = sum_ltsq * sum_gt_lt - sum_lt * sum_gt_lt;
        offset = offset_num / det;
#ifndef WIREDSYNC_BENCHMARK
        syncStruc sStruc = {offset, abs(inv_delta),(inv_delta>0?1:-1)};
        setSyncParam(sStruc);
#else
#ifdef ARCH_328P_ARDUINO
#else
        printf("inv_delta_den : %f, offset_num : %f\n",inv_delta_den.get_d(),offset_num.get_d());
        mpf_class delta = mpf_class(1)/mpf_class(inv_delta);
        printf("det : %e, invdelta : %f, delta : %e, offset : %f\n",det.get_d(), inv_delta.get_d(), delta.get_d(), offset.get_d());
#endif
#endif

        retVal = SYNC_SYNCHRONIZED;
    }
    else {
        retVal = SYNC_OUT_OF_SYNC;
    }
    if (reset){
        sum_ones=0;
        sum_lt=0;
        sum_ltsq=0;
        sum_gt_lt=0;
        sum_lt_gt_lt=0;
    }

    return retVal;
}
#ifndef WIREDSYNC_BENCHMARK
/* wiredSync_sendSignal : function that sends the synchronization signal.
 * "SyncParam" is reset in EVERY call to this function.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  None
 * Returned value :
 *  None
 */
void wiredSync_sendSignal(){
    syncStruc tmpStruc = getSyncParam();
    wiredSync_setSignal(WIREDSYNC_SIGNALISHERE);
    uint32_t begin = millis();
    while(millis()-begin < WIREDSYNC_LOWTIME);
    wiredSync_setSignal(WIREDSYNC_SIGNANOTHERE);
    uint32_t end=micros();
    tmpStruc.initialDelay = end-WIREDSYNC_INITIAL;
    setSyncParam(tmpStruc);
    updateSync();
}
#endif
