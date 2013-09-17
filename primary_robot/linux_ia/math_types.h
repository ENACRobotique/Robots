#ifndef _MATH_TYPES_H
#define _MATH_TYPES_H

// real number
typedef float sNum_t;

// 2D circle
typedef struct {
    sNum_t x;
    sNum_t y;
    sNum_t r;
} sCcl_t;

// 2D point
typedef struct {
    sNum_t x;
    sNum_t y;
} sPt_t;

// 2D vector
typedef sPt_t sVec_t;

// 2D segment
typedef struct {
    sPt_t p1;
    sPt_t p2;
} sSeg_t;

// 2D line (ax+by+c=0)
typedef struct {
    sNum_t a;
    sNum_t b;
    sNum_t c;

    unsigned char norm;
} sLin_t;

#endif

