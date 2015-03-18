
#include "math_ops.h"

extern "C"{
#include <math.h>
}


ERROR normVec(const sVec_t *v, sNum_t *n) {
    RET_IF_NOT_(v && n, ERR_BADPAR);

    *n = sqrt(v->x * v->x + v->y * v->y);

    return 0;
}

ERROR convPts2Vec(const sPt_t *a, const sPt_t *b, sVec_t *ab) {
    RET_IF_NOT_(a && b && ab, ERR_BADPAR);

    ab->x = b->x - a->x;
    ab->y = b->y - a->y;

    return 0;
}

ERROR convPts2Seg(const sPt_t *a, const sPt_t *b, sSeg_t *ab) {
    RET_IF_NOT_(a && b && ab, ERR_BADPAR);

    ab->p1 = *a;
    ab->p2 = *b;

    return 0;
}

ERROR sqdistPt2Pt(const sPt_t *p1, const sPt_t *p2, sNum_t *d2) {
    RET_IF_NOT_(p1 && p2 && d2, ERR_BADPAR);

    *d2 = (p2->x - p1->x) * (p2->x - p1->x) + (p2->y - p1->y) * (p2->y - p1->y);

    return 0;
}

ERROR distPt2Pt(const sPt_t *p1, const sPt_t *p2, sNum_t *d) {
    RET_IF_NOT_(p1 && p2 && d, ERR_BADPAR);

    *d = sqrt((p2->x - p1->x) * (p2->x - p1->x) + (p2->y - p1->y) * (p2->y - p1->y));

    return 0;
}

ERROR dotVecs(const sVec_t *v1, const sVec_t *v2, sNum_t *d) {
    RET_IF_NOT_(v1 && v2 && d, ERR_BADPAR);

    *d = v1->x * v2->x + v1->y * v2->y;

    return 0;
}

ERROR crossVecs(const sVec_t *v1, const sVec_t *v2, sNum_t *c) {
    RET_IF_NOT_(v1 && v2 && c, ERR_BADPAR);

    *c = v1->x * v2->y - v1->y * v2->x;

    return 0;
}

ERROR rotVec(const sNum_t theta, sVec_t *v) {
    sVec_t vc = *v;

    RET_IF_NOT_(v, ERR_BADPAR);

    v->x = vc.x * cos(theta) - vc.y * sin(theta);
    v->y = vc.x * sin(theta) + vc.y * cos(theta);

    return 0;
}

ERROR convPts2Line(const sPt_t *p1, const sPt_t *p2, int norm, sLin_t *l) {
    RET_IF_NOT_(p1 && p2 && l, ERR_BADPAR);

    l->norm = !!norm;

    if (!norm) {
        l->a = p2->y - p1->y;
        l->b = p1->x - p2->x;
    }
    else {
        sNum_t n;
        distPt2Pt(p1, p2, &n);

        l->a = (p2->y - p1->y) / n;
        l->b = (p1->x - p2->x) / n;
    }

    l->c = -l->a * p1->x - l->b * p1->y;

    return 0;
}

ERROR convVecPt2Line(const sVec_t *v, const sPt_t *p, int norm, sLin_t *l) {
    RET_IF_NOT_(v && p && l, ERR_BADPAR);

    l->norm = !!norm;

    if (!norm) {
        l->a = v->y;
        l->b = -v->x;
    }
    else {
        sNum_t n;
        normVec(v, &n);

        l->a = v->y / n;
        l->b = -v->x / n;
    }

    l->c = -l->a * p->x - l->b * p->y;

    return 0;
}

ERROR normLine(sLin_t *l) {
    sVec_t nv;
    sNum_t n;

    RET_IF_NOT_(l, ERR_BADPAR);

    if (!l->norm) {
        nv.x = -l->b;
        nv.y = l->a;

        normVec(&nv, &n);

        RET_IF_(n == 0, ERR_DIV0);

        l->a = l->a / n;
        l->b = l->b / n;
        l->c = l->c / n;

        l->norm = 1;
    }

    return 0;
}

ERROR sqdistPt2Seg(const sPt_t *p, const sSeg_t *s, sNum_t *d, sPt_t *h) {
    sNum_t l2, t;
    sVec_t p1p, p1p2;
    sPt_t proj;

    RET_IF_NOT_(p && s && d, ERR_BADPAR);

    sqdistPt2Pt(&s->p1, &s->p2, &l2);
    if (l2 == 0.) {
        return sqdistPt2Pt(p, &s->p1, d);
    }

    convPts2Vec(&s->p1, p, &p1p);
    convPts2Vec(&s->p1, &s->p2, &p1p2);

    dotVecs(&p1p, &p1p2, &t);

    if (t < 0.) {
        if (h)
            *h = s->p1;

        return sqdistPt2Pt(p, &s->p1, d);
    }

    if (t > l2) {
        if (h)
            *h = s->p2;

        return sqdistPt2Pt(p, &s->p2, d);
    }

    t = t / l2;

    proj.x = s->p1.x + t * p1p2.x;
    proj.y = s->p1.y + t * p1p2.y;

    if (h) {
        *h = proj;
    }

    return sqdistPt2Pt(p, &proj, d);
}

ERROR distPt2Line(const sPt_t *p, sLin_t *l, sNum_t *d, sPt_t *h) {
    RET_IF_NOT_(p && l, ERR_BADPAR);

    RET_IF_ERR_(normLine(l));

    if (d) {
        // for a normed line
        *d = l->a * p->x + l->b * p->y + l->c;
    }

    if (h) {
        // for a normed line
        h->x = l->b * (l->b * p->x - l->a * p->y) - l->a * l->c;
        h->y = -l->a * (l->b * p->x - l->a * p->y) - l->b * l->c;
    }

    return 0;
}

ERROR interLine2Line(const sLin_t *l1, const sLin_t *l2, int *nb, sPt_t *pt) {
    float det;

    RET_IF_NOT_(l1 && l2 && nb, ERR_BADPAR);

    if (!(det = l1->a * l2->b - l1->b * l2->a)) { //parallel
        *nb = 0;
    }
    else {
        if (pt) {
            pt->x = (1 / det) * (l1->b * l2->c - l1->c * l2->b);
            pt->y = (1 / det) * (l1->c * l2->a - l1->a * l2->c);
        }
        *nb = 1;
    }

    return 0;
}

ERROR interCircle2Line(const sLin_t *l, const sPt_t *c, sNum_t r, int *nb, sPt_t *pt1, sPt_t *pt2) {
    sNum_t a, b, d, det;
    sPt_t p1, p2;

    RET_IF_NOT_(l && c && nb, ERR_BADPAR);
    RET_IF_((l->a == 0) && (l->b == 0), ERR_BADPAR);

    if (l->a == 0) {
        a = l->b * l->b;
        b = -2 * c->x * l->b * l->b;
        d = l->b * l->b * (c->x * c->x + c->y * c->y - r * r) + 2 * l->b * l->c * c->x + l->c * l->c;
        if (((det = b * b - 4 * a * d) < 0) || (b == 0)) {
            *nb = 0;
            return 0;
        }
        else {
            p1.x = (-b - sqrt(det)) / (2 * a);
            p1.y = -l->c / l->b;
            p2.x = (-b + sqrt(det)) / (2 * a);
            p2.y = -l->c / l->b;
        }
    }
    else {
        a = (l->a * l->a + l->b * l->b);
        b = 2 * (l->b * l->c + l->a * l->b * c->x - l->a * l->a * c->y);
        d = l->a * l->a * (c->x * c->x + c->y * c->y - r * r) + 2 * l->a * l->c * c->x + l->c * l->c;
        if ((det = b * b - 4 * a * d) < 0) {
            *nb = 0;
            return 0;
        }
        else {
            p1.y = (-b - sqrt(det)) / (2 * a);
            p1.x = -(l->c + l->b * p1.y) / l->a;
            p2.y = (-b + sqrt(det)) / (2 * a);
            p2.x = -(l->c + l->b * p2.y) / l->a;
        }
    }

    if ((p1.x == p2.x) && (p1.y == p2.y)) {
        *nb = 1;
    }
    else {
        *nb = 2;
    }

    if (pt1) {
        *pt1 = p1;
    }

    if (pt2) {
        *pt2 = p2;
    }

    return 0;
}

ERROR testPtOnArc(const sPt_t *p1, const sPt_t *p2, const sPt_t *c, sNum_t r, sPt_t *p, int *ret) { //test if the point belongs on the part of circle FIXME test function
    sNum_t theta1, theta2, theta;

    RET_IF_NOT_(p1 && p2 && c && p && ret, ERR_BADPAR);

    if ((theta1 = atan2(p1->y - c->y, p1->x - c->x)) < 0) {
        theta1 += 2 * M_PI;
    }
    if ((theta2 = atan2(p2->y - c->y, p2->x - c->x)) < 0) {
        theta2 += 2 * M_PI;
    }
    if ((theta = atan2(p->y - c->y, p->x - c->x)) < 0) {
        theta += 2 * M_PI;
    }

    if (theta2 < theta1) {
        if ((theta < theta1) && (theta > theta2)) {
            *ret = 0;
            return 0;
        }
    }
    else {
        if ((theta < theta1) || (theta > theta2)) {
            *ret = 0;
            return 0;
        }
    }

    *ret = 1;
    return 0;
}

ERROR symPtprLine(const sPt_t *pt, sLin_t *l, sPt_t *ps) {
    sPt_t pp;

    RET_IF_NOT_(pt && l && ps, ERR_BADPAR);

    distPt2Line(pt, l, NULL, &pp);

    ps->x = pt->x - 2 * (pt->x - pp.x);
    ps->y = pt->y - 2 * (pt->y - pp.y);

    return 0;
}

ERROR testPtInZone(const sPt_t pz[], int nb, const sPt_t *pt, int *ret) { //FIXME if angle between two line more than 180°
    sLin_t l[nb];
    int sg_prev = 0, sg = 0;
    int i;

    RET_IF_NOT_(pz && pt && nb, ERR_BADPAR);

    for (i = 0; i < nb; i++) {
        convPts2Line(&pz[i], &pz[(i + 1) % (nb)], 0, &l[i]);
    }

    sign(l[0].a * pt->x + l[0].b * pt->y + l[0].c, &sg_prev);

    for (i = 1; i < nb; i++) {
        sign(l[i].a * pt->x + l[i].b * pt->y + l[i].c, &sg);
        if (sg != sg_prev) {
            *ret = 0;
            return 0;
        }
    }

    *ret = 1;
    return 0;
}

ERROR sign(const float x, int *sg) {

    RET_IF_NOT_(sg, ERR_BADPAR);

    *sg = (x > 0) - (x < 0);

    return 0;
}

ERROR projPtOnCircle(const sPt_t *c, const sNum_t r, sPt_t *p) {
    sVec_t v = { 0., 0. };
    sNum_t n;

    RET_IF_NOT_(c && p, ERR_BADPAR);
    distPt2Pt(c, p, &n);
    if(n < 0.1)
        return -1;

    convPts2Vec(c, p, &v);
    normVec(&v, &n);

    p->x = c->x + v.x * r / n;
    p->y = c->y + v.y * r / n;

    return 0;
}
