#ifndef ENCODER_H
#define ENCODER_H

#include <eint.h>

typedef enum {
    POSITIVE_IS_TRIGO, NEGATIVE_IS_TRIGO
} encoder_polarity_t;

typedef struct {
#ifdef ARCH_LPC21XX
    eEINT eint; // Id of interruption
    encoder_polarity_t pol;
    int nbticks;
    int cor_transmission; // Factor to correct the transmission
#elif defined(ARCH_X86_LINUX)
    // TODO
#endif
} encoder_t;

void encoder_init(encoder_t* e, encoder_polarity_t pol, eEINT eint, eEINT_PINASSIGN eint_pin, eEINT_MODE eint_type, eint_handler eint_h, int eint_prio);
int get_encoder(encoder_t* e);

}
#endif
