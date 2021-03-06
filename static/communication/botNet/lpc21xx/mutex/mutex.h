/*
 * mutex.h
 *
 *  Created on: 17 déc. 2013
 *      Author: ludo6431
 */

#ifndef MUTEX_H_
#define MUTEX_H_

#include <ime.h>

// pseudo-mutex for superBus central Buffer access management
// XXX: rename

static int irq_state = 0;

inline void bn_mutexLock(){
    irq_state = global_IRQ_disable();
}

inline void bn_mutexUnlock(){
    global_IRQ_restore(irq_state);
}

#endif /* MUTEX_H_ */
