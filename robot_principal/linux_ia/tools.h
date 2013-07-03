#ifndef _TOOLS_H
#define _TOOLS_H

#include <stdint.h>

#include "error.h"
#include "math_types.h"

// ==== definitions ====

/*
 *  physical obstacle 0, 1, 2, ..., N-1
 *  logical obstacle 0a, 0b, 1a, 1b, ... (notion of direction of rotation)
 *  rotation direction:
 *      a:clockwise
 *      b:counter-clockwise
 */

#define LOW_THR ((sNum_t)0.01)

#define R_SECU (5.)
#define R_ROBOT (20.)

#if 0
#   define X_MIN (R_ROBOT)
#   define X_MAX (300. - R_ROBOT)
#   define Y_MIN (R_ROBOT)
#   define Y_MAX (200. - R_ROBOT)
#else
#   define X_MIN (0.)
#   define X_MAX (300.)
#   define Y_MIN (0.)
#   define Y_MAX (200.)
#endif

#define OUT(x, y) ((x) > X_MAX || (x) < X_MIN || (y) > Y_MAX || (y) < Y_MIN)

// ==== common types ====

// an obstacle
typedef struct {
    sPt_t c;    // center of obstacle
    sNum_t r;   // radius

    uint8_t moved:4;
    uint8_t active:4;
} sObs_t;   // sizeof(sObs_t)=16

// a set of common tangents between two obstacles (from o1 to o2)
typedef struct {
    sSeg_t s1;  // first external (clock wise / a)
    sSeg_t s2;  // second external (counter clock wise / b)
    sSeg_t s3;  // first internal (counter clock wise / b)
    sSeg_t s4;  // second internal (clock wise / a)

    sNum_t d; // distance between obstacles
} sTgts_t;  // sizeof(sTgts_t)=68

// a tangent possibility between two obstacles (4 per pair of physical pobstacles)
typedef uint8_t sLnk_t; // sizeof(sLnk_t)=1

// an index of obstacle between 0:N-1
typedef int8_t iObs_t;
// an index of obstacle between 0:2N-1
typedef int8_t iABObs_t;

// between 0:2N-1
#define A(i) ((iABObs_t)( ((iObs_t)(i))<<1 ))
#define B(i) ((iABObs_t)( (((iObs_t)(i))<<1)+1 ))
// direction of a number in 0:2N-1
#define DIR(i) (((iABObs_t)(i))&1)

// between 0:N-1
#define O(i) ((iObs_t)( ((iABObs_t)(i))>>1 ))

// ==== global matrices ====

// number of physical obstacles (16)
#define N (15)
extern sObs_t obs[N]; // array of N physical obstacles (256B)
extern sTgts_t tgts[N][N];   // NxN tangents between physical obstacles (17kiB)
extern sLnk_t lnk[2*N][2*N]; // 2Nx2N links between logical obstacles (1kiB)
// NxN distances between obstacles
#define DIST(i, j) (tgts[(iObs_t)(i)][(iObs_t)(j)].d)

// ==== function prototypes ====

void    fill_tgts_lnk   ();

// functions on objects (0:N-1)
uint8_t o_check_segment (iObs_t o1, sSeg_t *s, iObs_t o2);
uint8_t o_check_arc     (iObs_t o1, sPt_t *p2_1, iObs_t o2, int dir, sPt_t *p2_3, iObs_t o3);
sNum_t  o_arc_len       (sPt_t *p2_1, iObs_t o2, int dir, sPt_t *p2_3);

// functions on directed objects (0:N-1)
static inline sSeg_t *tgt(iABObs_t o1, iABObs_t o2) {
    if(DIR(o1))
        return DIR(o2) ? &tgts[O(o1)][O(o2)].s2 : &tgts[O(o1)][O(o2)].s3;
    else
        return DIR(o2) ? &tgts[O(o1)][O(o2)].s4 : &tgts[O(o1)][O(o2)].s1;
}
static inline uint8_t check_arc(iABObs_t o1, iABObs_t o2, iABObs_t o3) {
    return o_check_arc(O(o1), &tgt(o1, o2)->p2, O(o2), DIR(o2), &tgt(o2, o3)->p1, O(o3));
}
static inline sNum_t arc_len(iABObs_t o1, iABObs_t o2, iABObs_t o3) {
    return o_arc_len(&tgt(o1, o2)->p2, O(o2), DIR(o2), &tgt(o2, o3)->p1);
}

#endif
