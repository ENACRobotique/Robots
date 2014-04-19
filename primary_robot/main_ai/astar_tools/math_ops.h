#ifndef _MATH_OPS_H
#define _MATH_OPS_H

#include <stdio.h>

#include "error.h"
#include "math_types.h"

inline ERROR normVec(const sVec_t *v, sNum_t *n);
inline ERROR convPts2Vec(const sPt_t *a, const sPt_t *b, sVec_t *ab);
inline ERROR convPts2Seg(const sPt_t *a, const sPt_t *b, sSeg_t *ab);
inline ERROR sqdistPt2Pt(const sPt_t *p1, const sPt_t *p2, sNum_t *d2);
inline ERROR distPt2Pt(const sPt_t *p1, const sPt_t *p2, sNum_t *d);
inline ERROR dotVecs(const sVec_t *v1, const sVec_t *v2, sNum_t *d);
inline ERROR crossVecs(const sVec_t *v1, const sVec_t *v2, sNum_t *c);
ERROR convVecPt2Line(const sVec_t *v, const sPt_t *p, int norm, sLin_t *l);
ERROR normLine(sLin_t *l);
ERROR sqdistPt2Seg(const sPt_t *p, const sSeg_t *s, sNum_t *d, sPt_t *h);
ERROR distPt2Line(const sPt_t *p, sLin_t *l, sNum_t *d);

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
