/*
 * lib_synchro_wire_arduino.c
 *
 *  Created on: 7 avr. 2015
 *      Author: quentin
 */

#include "Arduino.h"
#include "lib_synchro_wire_arduino.h"

int syncPin;

/* wiredSync_{sender,receiver}Init : init function for sender/receiver of the synchronization signal.
 * There MUST be exactly ONE sender, and possibly multiple receiver.
 */
void wiredSync_senderInit(int pin){
    syncPin = pin;
    pinMode(syncPin,OUTPUT);
    wiredSync_setSignal(WIREDSYNC_SIGNANOTHERE);
}
void wiredSync_receiverInit(int pin){
    syncPin = pin;
    pinMode(syncPin,INPUT_PULLUP);
}

/* wiredSync_signalPresent : low-level assert of presence of signal
 * Returned value :
 *   WIREDSYNC_SIGNALISHERE if the signal is present
 *   WIREDSYNC_SIGNANOTHERE otherwise
 */
int wiredSync_signalPresent(){
    return digitalRead(syncPin);
}

/* wiredSync_setSignal : low-level function used to set the signal value.
 * Argument :
 *  val : vale of the signal to send
 *
 */
void wiredSync_setSignal(int val){
    digitalWrite(syncPin,val);
}
