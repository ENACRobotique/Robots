/*
 * lib_synchro_wire.h
 *
 *  Created on: 5 avr. 2015
 *      Author: quentin
 */

#ifndef LIB_SYNCHRO_WIRE_H_
#define LIB_SYNCHRO_WIRE_H_

#include "lib_synchro.h"

#define WIREDSYNC_INITIAL 0     // time to set when we receive the first sync signal.
#define WIREDSYNC_LOWTIME 1000  // in ms
#define WIREDSYNC_MAXLOOP 200   // maximum main loop duration in ms
#define WIREDSYNC_DEBOUNCE (WIREDSYNC_LOWTIME-WIREDSYNC_MAXLOOP)


/* wiredSync_{sender,receiver}Init : init function for sender/receiver of the synchronization signal.
 * There MUST be exactly ONE sender, and possibly multiple receiver.
 */
void wiredSync_senderInit(int pin);
void wiredSync_receiverInit(int pin);

/* wiredSync_waitSignal : function that must be in the main loop, and wiats for the wired synchronization signal.
 * "SyncParam" is reset in EVERY TIME this signal is received.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  None
 * Returned value :
 *  SYNC_OUT_OF_SYNC while no synchronizing signal has been received
 *  SYNC_SYNCHRONIZED after the synchronizing signal has been received
 */
int wiredSync_waitSignal();

/* wiredSync_sendSignal : function that sends the synchronization signal.
 * "SyncParam" is reset in EVERY call to this function.
 * WILL BLOCK DURING SYNCHRONIZATION, blocking delay is at most WIREDSYNC_LOWTIME
 * Argument :
 *  None
 * Returned value :
 *  None
 */
void wiredSync_sendSignal();


#endif /* LIB_SYNCHRO_WIRE_H_ */
