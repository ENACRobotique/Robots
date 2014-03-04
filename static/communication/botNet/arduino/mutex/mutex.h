/*
 * mutex.h
 *
 *  Created on: 18 oct. 2013
 *      Author: quentin
 */


#ifdef ARCH_328P_ARDUINO

#ifndef MUTEX_H_
#define MUTEX_H_

#include "Arduino.h"

// XXX: rename

static volatile uint8_t SREG_save;

//pseudo-mutex for superBus central Buffer access management
inline void mutexLock(){
    SREG_save = SREG;
    noInterrupts();
}

inline void mutexUnlock(){
    SREG = SREG_save;
}

#endif /* MUTEX_H_ */

#endif // ARCH_328P_ARDUINO
