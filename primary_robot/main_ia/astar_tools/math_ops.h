#ifndef _MATH_OPS_H
#define _MATH_OPS_H

#include <stdio.h>

#include "error.h"
#include "math_types.h"

inline ERROR normVec(sVec_t *v, sNum_t *n);
inline ERROR convPts2Vec(sPt_t *a, sPt_t *b, sVec_t *ab);
inline ERROR convPts2Seg(sPt_t *a, sPt_t *b, sSeg_t *ab);
inline ERROR sqdistPt2Pt(sPt_t *p1, sPt_t *p2, sNum_t *d2);
inline ERROR distPt2Pt(sPt_t *p1, sPt_t *p2, sNum_t *d);
inline ERROR dotVecs(sVec_t *v1, sVec_t *v2, sNum_t *d);
inline ERROR crossVecs(sVec_t *v1, sVec_t *v2, sNum_t *c);
ERROR convVecPt2Line(sVec_t *v, sPt_t *p, int norm, sLin_t *l);
ERROR normLine(sLin_t *l);
ERROR sqdistPt2Seg(sPt_t *p, sSeg_t *s, sNum_t *d, sPt_t *h);
ERROR distPt2Line(sPt_t *p, sLin_t *l, sNum_t *d);

static inline void dumpVec(sVec_t *v, char *name) {
    printf("%s(%.2f,%.2f)\n", name, v->x, v->y);
}

static inline void dumpPt(sPt_t *p, char *name) {
    printf("%s(%.2f,%.2f)\n", name, p->x, p->y);
}

static inline void dumpSeg(sSeg_t *s, char *name) {
    printf("%s(%.2f,%.2f -> %.2f,%.2f)\n", name, s->p1.x, s->p1.y, s->p2.x, s->p2.y);
}

static inline void dumpNum(sNum_t n, char *name) {
    printf("%s=%.2f\n", name, n);
}

#endif
