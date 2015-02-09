#ifndef ENCODER_H
#define ENCODER_H

#include "eint.h"

typedef struct {
    eEINT eint; // Id of interruption
    int nbticks;
    int dir; // direction of rotation: 1 = trigo, -1 = no trigo
    int cor_transmission; // Factor to correct the transmission
} encoder_t;

void encoder_init(encoder_t* e, eEINT eint, eEINT_PINASSIGN eint_pin, eEINT_MODE eint_type, eint_handler eint_h, int eint_prio);

#endif
