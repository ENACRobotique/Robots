#ifndef _PERCEPTION_H
#define _PERCEPTION_H

#include "tools.h"

typedef struct {
// distances aux balises (m)
    float d1;
    float d2;
    float d3;

    float u_d1;
    float u_d2;
    float u_d3;

// angles inter-balises (rad)
    float a12;
    float a23;
    float a31;

    float u_a12;
    float u_a23;
    float u_a31;
} sPerception;

void simu_perception(sPt_t *x, sPerception *p);
void estim_incertitude(sPerception *p);
void bruite_perception(sPerception *p);

#endif

