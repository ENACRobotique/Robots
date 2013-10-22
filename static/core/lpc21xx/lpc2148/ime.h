/*
 * ime.h
 *
 *  Created on: 22 oct. 2013
 *      Author: ludo6431
 */

#ifndef IME_H_
#define IME_H_

int global_interrupts_set(int status);
#define global_interrupts_enable() global_interrupts_set(1)
#define global_interrupts_disable() global_interrupts_set(0)

#endif /* IME_H_ */
