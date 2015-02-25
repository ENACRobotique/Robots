#ifndef _TRIGO_H
#define _TRIGO_H

#include "params.h"

#define COS(a) iROUND(cos(((double)(a))/dSHIFT)*dSHIFT)
#define SIN(a) iROUND(sin(((double)(a))/dSHIFT)*dSHIFT)
#define ATAN2(y, x) iROUND(atan2(((double)(y))/dSHIFT, ((double)(x))/dSHIFT)*dSHIFT)
#define SQRT(v) iROUND(dSHIFT*sqrt((double)(v)/dSHIFT))

#endif

