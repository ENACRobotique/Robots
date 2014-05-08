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
ERROR convPts2Line(const sPt_t *p1, const sPt_t *p2, int norm, sLin_t *l);
ERROR convVecPt2Line(const sVec_t *v, const sPt_t *p, int norm, sLin_t *l);
ERROR normLine(sLin_t *l);
ERROR sqdistPt2Seg(const sPt_t *p, const sSeg_t *s, sNum_t *d, sPt_t *h);
ERROR distPt2Line(const sPt_t *p, sLin_t *l, sNum_t *d, sPt_t *h);
ERROR interLine2Line(const sLin_t *l1, const sLin_t *l2, int *nb, sPt_t *pt);
ERROR interCircle2Line(const sLin_t *l, const sPt_t *c, sNum_t r, int *nb, sPt_t *pt1, sPt_t *pt2);
ERROR testPtOnArc(const sPt_t *p1, const sPt_t *p2 ,const sPt_t *c, sNum_t r, sPt_t *p, int *ret);
ERROR symPtprLine(const sPt_t *pt, sLin_t *l, sPt_t *ps);
ERROR testPtInZone(const sPt_t pz[], int nb, const sPt_t *pt, int *ret);
ERROR signum(const float x, int *sg);

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
