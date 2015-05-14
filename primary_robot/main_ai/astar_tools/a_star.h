#ifndef _A_STAR_H
#define _A_STAR_H

#include "a_star_tools.h"

#include "GeometryTools.h"

typedef struct {
    Point2D<float> p1;
    Point2D<float> p2;
    astar::sObs_t obs;

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


void a_star(astar::iABObs_t start, astar::iABObs_t goal, sPath_t *path);

#endif
