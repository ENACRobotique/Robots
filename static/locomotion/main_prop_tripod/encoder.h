#ifndef ENCODER_H
#define ENCODER_H

#include "eint.h"

typedef struct {
    eEINT eint;
    int nbticks;
} encoder_t;

void encoder_init(encoder_t* e);

#endif
