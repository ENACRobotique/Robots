/*
 * lib_synchro_wire_arduino.h
 *
 *  Created on: 7 avr. 2015
 *      Author: quentin
 */

#ifndef LIB_SYNCHRO_WIRE_ARDUINO_H_
#define LIB_SYNCHRO_WIRE_ARDUINO_H_

#include "Arduino.h"

#define WIREDSYNC_SIGNALISHERE LOW
#define WIREDSYNC_SIGNANOTHERE HIGH

#ifdef __cplusplus
extern "C" {
#endif

/* wiredSync_{sender,receiver}Init : init function for sender/receiver of the synchronization signal.
 * There MUST be exactly ONE sender, and possibly multiple receiver.
 */
void wiredSync_senderInit(int pin);
void wiredSync_receiverInit(int pin);

/* wiredSync_signalPresent : low-level assert of presence of signal
 * Returned value :
 *   WIREDSYNC_SIGNALISHERE if the signal is present
 *   WIREDSYNC_SIGNANOTHERE otherwise
 */
int wiredSync_signalPresent();

/* wiredSync_setSignal : low-level function used to set the signal value.
 * Argument :
 *  val : vale of the signal to send
 *
 */
void wiredSync_setSignal(int val);

#ifdef __cplusplus
}
#endif

#endif /* LIB_SYNCHRO_WIRE_ARDUINO_H_ */
