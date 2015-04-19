/*
 * lib_synchro_wire.h
 *
 *  Created on: 5 avr. 2015
 *      Author: quentin
 *
 *
 *
 * One wire interruptionless synchronization.
 * Signal presence == wire to ground. (use WIRESYNC_SIGNALISHERE or WIRESYNC_SIGNANOTHERE defines)
 * Every node MUST call either wiredSync_senderInit(pin) or wiredSync_receiverInit(pin) during it's init phase.
 * Sender node must send its signal only when the two receiver are already potentially listening.
 */

#ifndef LIB_SYNCHRO_WIRE_H_
#define LIB_SYNCHRO_WIRE_H_

#include "lib_synchro.h"

#ifdef ARCH_328P_ARDUINO
#include "arduino/lib_synchro_wire_arduino.h"
#endif


#define WIREDSYNC_INITIAL 0     // time to set when we receive the first sync signal.
#define WIREDSYNC_LOWTIME 1000  // in ms
#define WIREDSYNC_MAXLOOP 200   // maximum main loop duration in ms
#define WIREDSYNC_DEBOUNCE (WIREDSYNC_LOWTIME-WIREDSYNC_MAXLOOP)

#if defined(WIREDSYNC_BENCHMARK) && defined(ARCH_X86_LINUX)
#include <gmpxx.h>
#endif

typedef mpz_class wsType_t; // because it is shorter than wiredSyncType_t, and to allow easy change in order to benchmark different solutions

/* wiredSync_waitSignal : function that must be in the main loop, and waits for the wired synchronization signal.
 * This function is to be called on the device which has NOT the reference clock.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  None
 * Returned value :
 *  SYNC_OUT_OF_SYNC while no synchronizing signal has been received
 *  SYNC_SYNCHRONIZED after the synchronizing signal has been received
 */
int wiredSync_waitSignal();

/* wiredSync_intermediateCompute :  records a new set of measures for the synchronization.
 * The actual computation is done in wiredSync_finalCompute.
 * This function is to be called on the device which has NOT the reference clock.
 * Arguments :
 *  gTime : global date of the event, i.e. time of the reference clock when the synchronizing event occurred
 *  lTime : local date of the same event (this one is pretty self-explaining)
 * Return value :
 *  none
 */
void wiredSync_intermediateCompute(wsType_t gTime, wsType_t lTime);

/* wiredSync_finalCompute : actual computation of the synchronization.
 * This function is to be called on the device which has NOT the reference clock.
 * Arguments :
 *  reset : if reset != 0, will reset the sums (i.e. delete previous measures)
 * Returned value :
 *  SYNC_OUT_OF_SYNC while no synchronization has been successful
 *  SYNC_SYNCHRONIZED after the synchronization is achieved, and syncParams updated
 */
int wiredSync_finalCompute(int reset);


/* wiredSync_sendSignal : function that sends the synchronization signal.
 * "SyncParam" is reset in EVERY call to this function.
 * This function is to be called on the device which has the reference clock.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  None
 * Returned value :
 *  None
 */
void wiredSync_sendSignal();

#endif /* LIB_SYNCHRO_WIRE_H_ */
