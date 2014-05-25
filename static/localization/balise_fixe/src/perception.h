#ifndef _PERCEPTION_H
#define _PERCEPTION_H

#include "tools.h"

typedef enum {
    BEACON_1=0,
    BEACON_2,
    BEACON_3,

    BEACON_AMOUNT
}eBeacon;   // beacon identifier

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

typedef struct {
    unsigned long deltaT;       // µs, delay between two laser small peaks
    unsigned long date;         // local µs, when was the laser recorded last
    unsigned long period;       // µs, MEASURED period (0 if not applicable).
    unsigned long u_date;
    eBeacon beacon;             // beacon identifier
}sMeasures;

sPerception calcPerception( sMeasures *mes1, sMeasures *mes2, sMeasures *mes3);

void simu_perception(sPt_t *x, sPerception *p);
void estim_incertitude(sPerception *p);
void bruite_perception(sPerception *p);

#endif

