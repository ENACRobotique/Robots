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

#define LOW_THR (0.01)

// ==== common types ====

// an obstacle
typedef struct {
    sPt_t c;    // center of obstacle
    sNum_t r;   // radius

    uint8_t moved;  // flag
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

// ==== global matrices ====

// number of physical obstacles (16)
#define N (5)
extern sObs_t obs[N]; // array of physical obstacles (256B)
extern sTgts_t tgts[N][N];   // tangents between physical obstacles (17kiB)
extern sLnk_t lnk[2*N][2*N]; // halfmatrix of 2Nx2N links between logical obstacles (1kiB)

// ==== function prototypes ====

uint8_t get_tangents(uint8_t _o1, uint8_t _o2);
uint8_t check_segment(uint8_t o1, uint8_t o2, sSeg_t *s);
void get_links();
uint8_t check_arc(uint8_t o1, sPt_t *p1, uint8_t o2, uint8_t o3, sPt_t *p3, uint8_t dir);
sNum_t arc_len(sPt_t *p1, uint8_t o2, sPt_t *p3, uint8_t dir);

#endif

