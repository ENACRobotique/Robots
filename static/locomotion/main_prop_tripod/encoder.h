#ifndef ENCODER_H
#define ENCODER_H

#include <eint.h>

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
#ifdef ARCH_LPC21XX
    eEINT eint; // Id of interruption
    int nbticks; // Updated when there is an interruption and reset on encoder_update() call
    int nbticks_cache; // Cached nbticks between two encoder_update() calls
#elif defined(ARCH_X86_LINUX)
    // TODO
#endif
} encoder_t;

void encoder_init   (encoder_t* e, eEINT eint, eEINT_PINASSIGN eint_pin, eEINT_MODE eint_type, eint_handler eint_h, int eint_prio);
void encoder_update (encoder_t* e);
int  encoder_get    (encoder_t* e);
void encoder_deinit (encoder_t* e);

#endif
