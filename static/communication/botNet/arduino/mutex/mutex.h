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

//pseudo-mutex for superBus central Buffer access management
inline void mutexLock(){
    noInterrupts();
}

inline void mutexUnlock(){
    interrupts();
}

#endif /* MUTEX_H_ */

#endif // ARCH_328P_ARDUINO
