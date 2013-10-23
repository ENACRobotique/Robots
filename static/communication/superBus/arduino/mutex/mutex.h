/*
 * mutex.h
 *
 *  Created on: 18 oct. 2013
 *      Author: quentin
 */

#ifndef MUTEX_H_
#define MUTEX_H_

#include "Arduino.h"


//pseudo-mutex for superBus central Buffer access management
inline void mutexLock(){
    noInterrupts();
}

inline void mutexUnlock(){
    interrupts();
}

#endif /* MUTEX_H_ */
