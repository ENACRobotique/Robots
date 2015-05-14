#ifndef ENCODER_H
#define ENCODER_H

#ifdef ARCH_LPC21XX
#include <eint.h>
#endif

#include "motor.h"

/**
 * Generic encoder reading for LPC2148
 * Relies on lpc_lib
 *
 * author: Ludovic Lacoste
 */

/**
 * Storage for an instance of encoder
 *   you must update manually the nbticks field in the interrupt service routine gave to the encoder_init function
 *   all other fields are private, use encoder_*() functions
 */
typedef struct {
    motor_t* m; // Associated motor (may be used for per-motor output adaptation (direction, correction factor, reduction ratio compensation, ...)
    int nbticks_cache; // Cached nbticks between two encoder_update() calls

#ifdef ARCH_LPC21XX
    eEINT eint; // Id of interruption
    int nbticks; // Updated when there is an interruption and reset on encoder_update() call
#elif defined(ARCH_X86_LINUX)
    unsigned int lastticksquery;
#endif
} encoder_t;

#ifdef ARCH_LPC21XX
void encoder_init   (encoder_t* e, motor_t* m, eEINT eint, eEINT_PINASSIGN eint_pin, eEINT_MODE eint_type, eint_handler eint_h, int eint_prio);
#elif defined(ARCH_X86_LINUX)
void encoder_init   (encoder_t* e, motor_t* m);
#endif
void encoder_update (encoder_t* e);
int  encoder_get    (encoder_t* e);
void encoder_deinit (encoder_t* e);

#endif
