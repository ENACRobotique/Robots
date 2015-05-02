/*
 * lib_synchro_wire.c
 *
 *  Created on: 5 avr. 2015
 *      Author: quentin
 */

#include "timeout.h"
#include "lib_synchro_wire.h"
#if defined(DEBUG_SYNC_WIRE) && !defined(WIREDSYNC_BENCHMARK)
#include "bn_debug.h"
#include "params.h"
#endif
#if defined(WIREDSYNC_BENCHMARK) && defined(ARCH_X86_LINUX)
#include <stdio.h>
#include <iostream>
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

#ifdef ARCH_328P_ARDUINO
/* wiredSync_waitSignal : function that must be in the main loop, and waits for the wired synchronization signal.
 * This function is to be called on the device which has NOT the reference clock.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  reset : if != 0, resets the index count (starts new synchronization)
 * Returned value :
 *  if >=0 : current sample index
 *  if <= 0 : no significant signal received
 */
int wiredSync_waitSignal(int reset){
    static int sampleIndex = -1;
    static uint32_t prevReceived = 0;
    if (reset){
        sampleIndex = -1;
        prevReceived = 0;
    }
    uint32_t begin = micros();
    // if signal is here
    while (wiredSync_signalPresent()==WIREDSYNC_SIGNALISHERE);
    uint32_t end = micros();
    // if signal stayed here for more than debounce time, record sample
    if ((end-begin) > WIREDSYNC_DEBOUNCE && (end-begin) < WIREDSYNC_LOWTIME*3){
#if WIREDSYNC_DEBOUNCE > (WIREDSYNC_LOWTIME*3)
#error "will not work. Debounce too slow, or lowtime too fast"
#endif
        if (prevReceived) {
            sampleIndex += (end - prevReceived + WIREDSYNC_PERIOD/4)/WIREDSYNC_PERIOD; // keep track of the indexes (takes into account missed ones,with a signal up to WIREDSYNC_PERIOD/4 early)
        }
        else {
            sampleIndex = 0;
        }
        prevReceived = end;

        wiredSync_intermediateCompute(sampleIndex*WIREDSYNC_PERIOD,end);
#if defined(DEBUG_SYNC_WIRE) && !defined(WIREDSYNC_BENCHMARK)
#if MYADDRX == ADDRX_MOBILE_1
        delay(20);
        bn_printfDbg(", mob1, %d, %lu, %lu",sampleIndex, end, sampleIndex*WIREDSYNC_PERIOD);
#endif
#if MYADDRX == ADDRX_MOBILE_2
        delay(40);
        bn_printfDbg(", mob2, %d, %lu, %lu\n",sampleIndex, end , sampleIndex*WIREDSYNC_PERIOD);
#endif
#endif
        return sampleIndex;
    }
    return -1;
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
        offset_num = sum_ltsq * sum_gt_lt - sum_lt * sum_lt_gt_lt;
        offset = offset_num / det;
#ifndef WIREDSYNC_BENCHMARK
        syncStruc sStruc = {static_cast<int32_t>(offset), static_cast<uint32_t>(abs(inv_delta)),(inv_delta>0?1:-1)};
        setSyncParam(sStruc);
#ifdef DEBUG_SYNC_WIRE
#if MYADDRX == ADDRX_MOBILE_1
        delay(20);
        bn_printfDbg(", mob1 end, %d, %ld, %lu\n",(int)sum_ones, static_cast<int32_t>(offset), static_cast<uint32_t>(abs(inv_delta)));
#endif
#if MYADDRX == ADDRX_MOBILE_2
        delay(40);
        bn_printfDbg(", mob2 end, %d, %ld, %lu\n",(int)sum_ones, static_cast<int32_t>(offset), static_cast<uint32_t>(abs(inv_delta)));
#endif
#endif
#else
#ifdef ARCH_328P_ARDUINO
#else
        wsType_t delta = 1/inv_delta;
        std::cout << "sum_gt_lt , sum_lt , sum_lt_gt_lt , sum_ltsq , sum_ones , inv_delta_den , offset_num, det ,";
        std::cout << "sum_ones*sum_lt_gt_lt , sum_ltsq * sum_ones , sum_lt*sum_lt , sum_lt*sum_gt_lt , sum_ltsq * sum_gt_lt , sum_lt * sum_gt_lt," ;
        std::cout << "inv_delta,delta,offset" << std::endl;
        std::cout << sum_gt_lt << "," << sum_lt << "," << sum_lt_gt_lt <<  "," << sum_ltsq << "," << sum_ones << "," << inv_delta_den << "," << offset_num<< "," << det << ",";
        std::cout << sum_ones*sum_lt_gt_lt << "," << sum_ltsq * sum_ones << "," << sum_lt*sum_lt << ","<< sum_lt*sum_gt_lt << "," << sum_ltsq * sum_gt_lt << "," << sum_lt * sum_gt_lt << ",";
        std::cout << inv_delta << "," << delta << "," << offset<< std::endl;
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
#ifdef ARCH_328P_ARDUINO
/* wiredSync_sendSignal : function that sends the synchronization signal.
 * This function is to be called on the device which has the reference clock.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  reset : if != 0, resets the index count (starts new synchronization)
 * Returned value :
 *  1 while there is still something to send
 *  -1 if all the signals have been sent
 */
int wiredSync_sendSignal(int reset){
    static uint32_t prevSignal=0;
    static int nbSamples=WIREDSYNC_NBSAMPLES; // number of samples left
    if (reset){
        prevSignal = 0;
        nbSamples = WIREDSYNC_NBSAMPLES;
    }
    if (nbSamples && (micros() - prevSignal) > (WIREDSYNC_PERIOD - WIREDSYNC_LOWTIME)){
        wiredSync_setSignal(WIREDSYNC_SIGNALISHERE);
        if (!prevSignal) delay(2 * WIREDSYNC_LOWTIME / 1000);   // we must force the waiting for the first call. Given that everything in this mechanism works with active waiting, using a delay() is not that bad. Why delay and not delayMicrosecond ? Take a look at the prototype of delayMicrosecond... Also, wait 2 time the normal duration is to ensure that every receiver catches the first sample (critical). Why exactly 2 ? I don't know.
        else while(micros()-prevSignal < WIREDSYNC_PERIOD);     // in the other cases, wait until exactly one period has elapsed since last signal
        wiredSync_setSignal(WIREDSYNC_SIGNANOTHERE);
        uint32_t end=micros();
        if (prevSignal) prevSignal += WIREDSYNC_PERIOD;
        else prevSignal = end;

        // set the "zero" right after the first signal
        if (nbSamples == WIREDSYNC_NBSAMPLES) {
            syncStruc tmpStruc = getSyncParam();
            tmpStruc.initialDelay = end-WIREDSYNC_INITIAL;
            setSyncParam(tmpStruc);
            updateSync();
        }
#if defined(DEBUG_SYNC_WIRE) && !defined(WIREDSYNC_BENCHMARK)
        bn_printfDbg("tur, %d, %lu", WIREDSYNC_NBSAMPLES - nbSamples, end);
#endif
        nbSamples --;
    }
    if (nbSamples) return 1;
    else return -1;
}
#endif
