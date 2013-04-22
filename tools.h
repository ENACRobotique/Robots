#ifndef _TOOLS_H
#define _TOOLS_H

#include "error.h"
#include "math_types.h"

typedef struct {
    sCcl_t *o1; // first obstacle
    sCcl_t *o2; // second obstacle
    sNum_t O1O2; // distance between obstacles

    sSeg_t s1;  // first external (clock wise)
    sSeg_t s2;  // second external (counter clock wise)
    sSeg_t s3;  // first internal (counter clock wise)
    sSeg_t s4;  // second internal (clock wise)

    unsigned char nb;   // number of tangents
} sTgts_t;

ERROR get_tangents(sCcl_t *o1, sCcl_t *o2, sTgts_t *out);
ERROR check_segment(sCcl_t *list, unsigned int n, int excl1, int excl2, sSeg_t *s, int *ok);
ERROR check_arc(sCcl_t *list, unsigned int n, int excl1, int excl2, int arc, sPt_t *p1, sPt_t *p2, int dir, int *ok);
ERROR arc_len(sPt_t *p1, sPt_t *p2, sPt_t *p3, sNum_t r, int dir, sNum_t *l);

// half matrix list (without diag): n*(n-1)/2 elements
#define HM_SIZE(n) (( (n)*((n)-1) )>>1)
#define HM_IDX(i, j, n) ((j) - 1 + (( (i)*(((n)<<1) - (i) - 3) )>>1))

#endif

