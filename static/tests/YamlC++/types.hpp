/*
 * types.h
 *
 *  Created on: 4 oct. 2014
 *      Author: seb
 */

#ifndef TYPES_H_
#define TYPES_H_

#include <stdint.h>
#include <math.h>
typedef float sNum_t;

//typedef struct sPt_t sPt_t;
struct sPt_t {
    sNum_t x;
    sNum_t y;

    sNum_t getDistTo(sPt_t& other){
        return sqrt((x - other.x)*(x - other.x) + (y - other.y)*(y - other.y));
    }
};

typedef struct {
    sPt_t c;    // center of obstacle
    sNum_t r;   // radius

    uint8_t moved:4;
    uint8_t active:4;
    uint8_t state; //0 free
} sObs_t;   // sizeof(sObs_t)=16

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

#endif /* TYPES_H_ */
