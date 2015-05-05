#ifndef TOOLS_H
#define TOOLS_H

// Basic math operations
#define SIGN(a) ((a)>=0 ? 1 : -1)

#ifndef BIT
#define BIT(b) (1<<(b))
#endif


#include "params.h"

#define __USE_GNU
#include <math.h>

#define COS(a) iROUND(cos(((double)(a))/dASHIFT)*dSHIFT)
#define SIN(a) iROUND(sin(((double)(a))/dASHIFT)*dSHIFT)
#define ATAN2(y, x) iROUND(atan2(((double)(y))/dSHIFT, ((double)(x))/dSHIFT)*dASHIFT)
#define SINCOS(a, s, c) do{ double sn, cs; sincos(((double)(a))/dASHIFT, &sn, &cs); (*(s)) = iROUND(sn*dSHIFT); (*(c) = iROUND(cs*dSHIFT)); }while(0)

#define SQRT(v) iROUND(dSHIFT*sqrt((double)(v)/dSHIFT))

#endif
