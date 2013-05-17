#ifndef _A_STAR_H
#define _A_STAR_H

#include "tools.h"

#define NOELT ((iABObs_t)-1)

typedef struct {
    sPt_t p1;
    sPt_t p2;
    sObs_t obs;
    
    sNum_t arc_len;
    sNum_t seg_len;
       
    unsigned short sid;
} sTrajEl_t;

typedef struct {
    sNum_t dist;
    unsigned short tid;
    
    unsigned int path_len;
    sTrajEl_t *path;
} sPath_t;

void a_star(iABObs_t start, iABObs_t goal, sPath_t *path);

#endif

