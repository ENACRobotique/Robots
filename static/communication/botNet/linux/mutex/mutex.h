/*
 * mutex.h
 *
 *  Created on: 18 oct. 2013
 *      Author: quentin
 */

#ifdef ARCH_X86_LINUX

#ifndef MUTEX_H_
#define MUTEX_H_

// warning, this is not a real mutex, use it only with botnet

//pseudo-mutex for superBus central Buffer access management

inline void mutexLock(){
    return;
}

inline void mutexUnlock(){
    return;
}

#endif /* MUTEX_H_ */

#endif // ARCH_X86_LINUX
