/*
 * ime.h
 *
 *  Created on: 22 oct. 2013
 *      Author: ludo6431
 */

#ifndef IME_H_
#define IME_H_

int global_IRQ_set(int status);
#define global_IRQ_enable() global_IRQ_set(1)
#define global_IRQ_disable() global_IRQ_set(0)
#define global_IRQ_restore(e) global_IRQ_set(e)

int global_FIQ_set(int status);
#define global_FIQ_enable() global_FIQ_set(1)
#define global_FIQ_disable() global_FIQ_set(0)
#define global_FIQ_restore(e) global_FIQ_set(e)

#endif /* IME_H_ */
