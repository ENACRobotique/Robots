/*
 * lib_synchro_wire.c
 *
 *  Created on: 5 avr. 2015
 *      Author: quentin
 */

#include "timeout.h"
#include "lib_synchro_wire.h"

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
    while (wiredSync_signalPresent());
    // if signal stayed here for more than debounce time, update initial time delay.
    uint32_t end = micros();
    if (end-begin > WIREDSYNC_DEBOUNCE){
        int32_t offset = end - WIREDSYNC_INITIAL;
        tmpStruc.initialDelay = offset;
        setSyncParam(tmpStruc);
        synchronized = 1;
    }
    return (synchronized?SYNC_SYNCHRONIZED:SYNC_OUT_OF_SYNC);
}

/* wiredSync_sendSignal : function that sends the synchronization signal.
 * "SyncParam" is reset in EVERY call to this function.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  None
 * Returned value :
 *  None
 */
void wiredSync_sendSignal(){
    uint32_t sw=0;
    syncStruc tmpStruc = getSyncParam();
    wiredSync_sendSignal(WIREDSYNC_SIGNALISHERE);
    while(testTimeout(WIREDSYNC_LOWTIME,&sw));
    wiredSync_setSignal(WIREDSYNC_SIGNANOTHERE);
    uint32_t end=micros();
    tmpStruc.initialDelay = end-WIREDSYNC_INITIAL;
    setSyncParam(tmpStruc);
}
