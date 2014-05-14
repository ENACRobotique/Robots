/*
 * mutex.h
 *
 *  Created on: 13 mai 2014
 *      Author: quentin
 */

#ifndef MUTEX_H_
#define MUTEX_H_

#ifdef ARCH_LM4FXX

#include "inc/hw_types.h"
#include "driverlib/interrupt.h"

static volatile tBoolean SREG_save;

//pseudo-mutex for superBus central Buffer access management
inline void mutexLock(){
    SREG_save = IntMasterDisable();
}

inline void mutexUnlock(){
    if(SREG_save) IntMasterEnable();
}

#endif

#endif /* MUTEX_H_ */
