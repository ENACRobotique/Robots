#include <math.h>

#include "math_ops.h"

inline ERROR normVec(sVec_t *v, sNum_t *n) {
    RET_IF_NOT_(v && n, ERR_BADPAR);

    *n = sqrt(v->x*v->x + v->y*v->y);

    return 0;
}

inline ERROR convPts2Vec(sPt_t *a, sPt_t *b, sVec_t *ab) {
    RET_IF_NOT_(a && b && ab, ERR_BADPAR);

    ab->x = b->x - a->x;
    ab->y = b->y - a->y;

    return 0;
}

inline ERROR sqdistPt2Pt(sPt_t *p1, sPt_t *p2, sNum_t *d2) {
    RET_IF_NOT_(p1 && p2 && d2, ERR_BADPAR);

    *d2 = (p2->x - p1->x)*(p2->x - p1->x) + (p2->y - p1->y)*(p2->y - p1->y);

    return 0;
}

inline ERROR distPt2Pt(sPt_t *p1, sPt_t *p2, sNum_t *d) {
    RET_IF_NOT_(p1 && p2 && d, ERR_BADPAR);

    *d = sqrt((p2->x - p1->x)*(p2->x - p1->x) + (p2->y - p1->y)*(p2->y - p1->y));

    return 0;
}

inline ERROR dotVecs(sVec_t *v1, sVec_t *v2, sNum_t *d) {
    RET_IF_NOT_(v1 && v2 && d, ERR_BADPAR);

    *d = v1->x*v2->x + v1->y*v2->y;

    return 0;
}

inline ERROR crossVecs(sVec_t *v1, sVec_t *v2, sNum_t *c) {
    RET_IF_NOT_(v1 && v2 && c, ERR_BADPAR);

    *c = v1->x*v2->y - v1->y*v2->x;

    return 0;
}

ERROR convVecPt2Line(sVec_t *v, sPt_t *p, int norm, sLin_t *l) {
    RET_IF_NOT_(v && p && l, ERR_BADPAR);

    l->norm = !!norm;

    if(!norm) {
        l->a = v->y;
        l->b = -v->x;
    }
    else {
        sNum_t n;
        normVec(v, &n);

        l->a = v->y/n;
        l->b = -v->x/n;
    }

    l->c = -l->a*p->x -l->b*p->y;

    return 0;
}

ERROR sqdistPt2Seg(sPt_t *p, sSeg_t *s, sNum_t *d, sPt_t *h) {
    sNum_t l2, t;
    sVec_t p1p, p1p2;
    sPt_t proj;

    RET_IF_NOT_(p && s && d, ERR_BADPAR);

    sqdistPt2Pt(&s->p1, &s->p2, &l2);
    if(l2 == 0.)
        return sqdistPt2Pt(p, &s->p1, d);

    convPts2Vec(&s->p1, p, &p1p);
    convPts2Vec(&s->p1, &s->p2, &p1p2);

    dotVecs(&p1p, &p1p2, &t);

    if(t < 0.) {
        if(h)
            *h = s->p1;

        return sqdistPt2Pt(p, &s->p1, d);
    }

    if(t > l2) {
        if(h)
            *h = s->p2;

        return sqdistPt2Pt(p, &s->p2, d);
    }

    t = t/l2;

    proj.x = s->p1.x + t*p1p2.x;
    proj.y = s->p1.y + t*p1p2.y;

    if(h)
        *h = proj;

    return sqdistPt2Pt(p, &proj, d);
}

