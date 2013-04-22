#include <math.h>
#include "math_ops.h"

#include "tools.h"

#define LOW_THR (0.1)

ERROR get_tangents(sCcl_t *o1, sCcl_t *o2, sTgts_t *out) {
    sVec_t o1o2, t, n;
    sNum_t st, ct;

    RET_IF_NOT_(o1 && o2 && out, ERR_BADPAR);

    out->o1 = o1;
    out->o2 = o2;

    o1o2.x = o2->x - o1->x;
    o1o2.y = o2->y - o1->y;

    out->O1O2 = sqrt(o1o2.x*o1o2.x + o1o2.y*o1o2.y);

    if(out->O1O2 < 2*LOW_THR) {
        out->nb = 0;

        return 0;
    }
    else if(o1->r < LOW_THR && o2->r < LOW_THR) {
        out->nb = 1;

        out->s1.p1.x = o1->x;
        out->s1.p1.y = o1->y;
        out->s1.p2.x = o2->x;
        out->s1.p2.y = o2->y;

        return 0;
    }
    else if(o1->r + out->O1O2 < o2->r + 4*LOW_THR || o2->r + out->O1O2 < o1->r + 4*LOW_THR) {
        out->nb = 0;

        return 0;
    }

    t.x = o1o2.x/out->O1O2;
    t.y = o1o2.y/out->O1O2;

    n.x = -t.y;
    n.y = t.x;

    if(out->O1O2 < o1->r + o2->r + 4*LOW_THR || o1->r < LOW_THR || o2->r < LOW_THR) {
        out->nb = 2;
    }
    else {
        out->nb = 4;

        // 2 internal tangents
        st = (o1->r + o2->r)/out->O1O2;
        ct = sqrt(1 - st*st);

        out->s3.p1.x = o1->x - o1->r*(-st*t.x + ct*n.x);
        out->s3.p1.y = o1->y - o1->r*(-st*t.y + ct*n.y);

        out->s4.p1.x = o1->x - o1->r*(-st*t.x - ct*n.x);
        out->s4.p1.y = o1->y - o1->r*(-st*t.y - ct*n.y);

        out->s3.p2.x = o2->x + o2->r*(-st*t.x + ct*n.x);
        out->s3.p2.y = o2->y + o2->r*(-st*t.y + ct*n.y);

        out->s4.p2.x = o2->x + o2->r*(-st*t.x - ct*n.x);
        out->s4.p2.y = o2->y + o2->r*(-st*t.y - ct*n.y);
    }

    // 2 external tangents
    st = (o2->r - o1->r)/out->O1O2;
    ct = sqrt(1 - st*st);

    out->s1.p1.x = o1->x + o1->r*(-st*t.x + ct*n.x);
    out->s1.p1.y = o1->y + o1->r*(-st*t.y + ct*n.y);

    out->s2.p1.x = o1->x + o1->r*(-st*t.x - ct*n.x);
    out->s2.p1.y = o1->y + o1->r*(-st*t.y - ct*n.y);

    out->s1.p2.x = o2->x + o2->r*(-st*t.x + ct*n.x);
    out->s1.p2.y = o2->y + o2->r*(-st*t.y + ct*n.y);

    out->s2.p2.x = o2->x + o2->r*(-st*t.x - ct*n.x);
    out->s2.p2.y = o2->y + o2->r*(-st*t.y - ct*n.y);

    return 0;
}

ERROR check_segment(sCcl_t *list, unsigned int n, int excl1, int excl2, sSeg_t *s, int *ok) {
    int i;
    sPt_t p;
    sNum_t d;

    RET_IF_NOT_(s && list && ok, ERR_BADPAR);

    *ok = 1;

    for(i = 0; i < n; i++) {
        if(i == excl1 || i == excl2 || list[i].r == 0.)
            continue;

        p.x = list[i].x;
        p.y = list[i].y;

        sqdistPt2Seg(&p, s, &d, NULL);

        if(d < list[i].r*list[i].r) {
            *ok = 0;
            break;
        }
    }

    return 0;
}

ERROR check_arc(sCcl_t *list, unsigned int n, int excl1, int excl2, int arc, sPt_t *p1, sPt_t *p2, int dir, int *ok) {
    int i;
    sPt_t c, p;
    sVec_t v1, v2;
    sLin_t l1, l2;
    sNum_t sc1, sc2, cross, d;

    RET_IF_NOT_(list && p1 && p2 && ok, ERR_BADPAR);

    c.x = list[arc].x;
    c.y = list[arc].y;

    v1.x = p1->x - c.x;
    v1.y = p1->y - c.y;
    convVecPt2Line(&v1, &c, 0, &l1);

    v2.x = p2->x - c.x;
    v2.y = p2->y - c.y;
    convVecPt2Line(&v2, &c, 0, &l2);

    crossVecs(&v1, &v2, &cross);

    if(!dir) {  // clock wise
        for(i = 0; i < n; i++) {
            if(i == excl1 || i == excl2 || i == arc || list[i].r == 0.)
                continue;

            p.x = list[i].x;
            p.y = list[i].y;

            sc1 = l1.a*p.x + l1.b*p.y + l1.c;
            sc2 = l2.a*p.x + l2.b*p.y + l2.c;

            if(cross < 0) {
                if(sc1 < 0 || sc2 > 0)
                    continue;
            }
            else {
                if(sc1 < 0 && sc2 > 0)
                    continue;
            }

            sqdistPt2Pt(&p, &c, &d);

            if(d < (list[i].r + list[arc].r)*(list[i].r + list[arc].r)) {
                *ok = 0;
                break;
            }
        }
    }
    else {  // counter clock wise
        for(i = 0; i < n; i++) {
            if(i == excl1 || i == excl2 || i == arc || list[i].r == 0.)
                continue;

            p.x = list[i].x;
            p.y = list[i].y;

            sc1 = l1.a*p.x + l1.b*p.y + l1.c;
            sc2 = l2.a*p.x + l2.b*p.y + l2.c;

            if(cross < 0) {
                if(sc1 > 0 && sc2 < 0)
                    continue;
            }
            else {
                if(sc1 > 0 || sc2 < 0)
                    continue;
            }

            sqdistPt2Pt(&p, &c, &d);

            if(d < (list[i].r + list[arc].r)*(list[i].r + list[arc].r)) {
                *ok = 0;
                break;
            }
        }
    }

    return 0;
}

ERROR arc_len(sPt_t *p1, sPt_t *p2, sPt_t *p3, sNum_t r, int dir, sNum_t *l) {
    sVec_t v1, v2;
    sNum_t d, c, t;

    RET_IF_NOT_(p1 && p2 && p3 && l, ERR_BADPAR);

    convPts2Vec(p2, p1, &v1);
    convPts2Vec(p2, p3, &v2);

    dotVecs(&v1, &v2, &d);
    crossVecs(&v1, &v2, &c);

    t = acos(d/(r*r));    

    if(!dir) {  // clock wise
        if(c > 0) {
            t += M_PI;
        }
    }
    else {  // counter clock wise
        if(c < 0) {
            t += M_PI;
        }
    }

    *l = t*r;

    return 0;
}

