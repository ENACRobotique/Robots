#ifndef _OPTIM_H
#define _OPTIM_H

#include "tools.h"
#include "perception.h"

#define Z_CRIT 18

extern unsigned int nb_cri;

float critere(sPt_t *x, sPerception *p);

#endif

