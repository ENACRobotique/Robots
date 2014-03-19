#include <math.h>
#include <stdio.h>
#include "math_ops.h"

#include "tools.h"

#define CHECK_LIMITS

// array of physical obstacles (256B)
sObs_t obs[] = {
    // départ
    {{0., 0.}, 0., 1, 1},
    // arbres
    {{0.  , 70. }, R_ROBOT+15, 1, 0}, // 1
    {{70. , 0.  }, R_ROBOT+15, 1, 0},
    {{230., 0.  }, R_ROBOT+15, 1, 0},
    {{300., 70. }, R_ROBOT+15, 1, 0},
    // bac fruit
    {{55. , 185.}, R_ROBOT+15, 1, 1}, // 5
    {{75. , 185.}, R_ROBOT+15, 1, 1},
    {{95. , 185.}, R_ROBOT+15, 1, 1},
    {{42. , 172.}, R_ROBOT+ 4, 1, 1},
    {{108., 172.}, R_ROBOT+ 4, 1, 1},
    {{205., 185.}, R_ROBOT+15, 1, 1},
    {{225., 185.}, R_ROBOT+15, 1, 1},
    {{245., 185.}, R_ROBOT+15, 1, 1},
    {{192., 172.}, R_ROBOT+ 4, 1, 1},
    {{258., 172.}, R_ROBOT+ 4, 1, 1},
    // foyers
    {{0.  , 0.  }, R_ROBOT+25, 1, 1}, // 15
    {{300., 0.  }, R_ROBOT+25, 1, 1},
    {{150., 95. }, R_ROBOT+15, 1, 1},
    // torches mobile
    {{90. , 90. }, R_ROBOT+8, 1, 1}, // 18
    {{210., 90. }, R_ROBOT+8, 1, 1},
    // torches fixe
    {{0.  , 120.}, R_ROBOT+5, 1, 1}, // 20
    {{130., 0.  }, R_ROBOT+5, 1, 1},
    {{170., 0.  }, R_ROBOT+5, 1, 1},
    {{300., 120.}, R_ROBOT+5, 1, 1},
    // feux
    {{40. , 90. }, R_ROBOT+7, 1, 1}, // 24
    {{90. , 40. }, R_ROBOT+7, 1, 1},
    {{90. , 140.}, R_ROBOT+7, 1, 1},
    {{210., 40. }, R_ROBOT+7, 1, 1},
    {{210., 140.}, R_ROBOT+7, 1, 1},
    {{260., 90. }, R_ROBOT+7, 1, 1},
    {{0.  , 120.}, R_ROBOT+7, 1, 1},
    {{130., 0.  }, R_ROBOT+7, 1, 1},
    {{170., 0.  }, R_ROBOT+7, 1, 1},
    {{300., 120.}, R_ROBOT+7, 1, 1},
    {{90. , 90. }, R_ROBOT+7, 1, 1},
    {{90. , 90. }, R_ROBOT+7, 1, 1},
    {{90. , 90. }, R_ROBOT+7, 1, 1},
    {{210., 90. }, R_ROBOT+7, 1, 1},
    {{210., 90. }, R_ROBOT+7, 1, 1},
    {{210., 90. }, R_ROBOT+7, 1, 1},
    // arrivée
    {{0. , 0.}, 0, 0, 1} // 40
};

// tangents between physical obstacles (17kiB)
sTgts_t tgts[N][N];
// A* elements
sASEl_t aselts[N*2][N*2];


static uint8_t fill_tgts(iObs_t _o1, iObs_t _o2) { // private function, _o1 < _o2
    sVec_t o1o2, t, n;
    sNum_t st, ct;
    sTgts_t *out = &tgts[_o1][_o2], *out_s = &tgts[_o2][_o1];
    sObs_t *o1 = &obs[_o1];
    sObs_t *o2 = &obs[_o2];

    convPts2Vec(&o1->c, &o2->c, &o1o2);

    normVec(&o1o2, &out->d);
        out_s->d = out->d;

    if(!o1->active || !o2->active)
        return 0;

    if(out->d < 2*LOW_THR) {
//        printf("concentric %u/%u\n", _o1, _o2);
        return 0;
    }
    else if(o1->r < LOW_THR && o2->r < LOW_THR) {
        out->s1.p1 = o1->c;
        out->s1.p2 = o2->c;
            out_s->s1.p1 = out->s1.p2;
            out_s->s1.p2 = out->s1.p1;

//        printf("straight line %u/%u\n", _o1, _o2);
        return 1;
    }
    else if(o1->r + out->d < o2->r + 4*LOW_THR || o2->r + out->d < o1->r + 4*LOW_THR) {
//        printf("inside %u/%u\n", _o1, _o2);
        return 0;
    }

    t.x = o1o2.x/out->d;
    t.y = o1o2.y/out->d;

    n.x = -t.y;
    n.y = t.x;

    // 2 external tangents
    st = (o2->r - o1->r)/out->d;
    ct = sqrt(1 - st*st);

    out->s1.p1.x = o1->c.x + o1->r*(-st*t.x + ct*n.x);
    out->s1.p1.y = o1->c.y + o1->r*(-st*t.y + ct*n.y);
        out_s->s2.p2 = out->s1.p1;

    out->s2.p1.x = o1->c.x + o1->r*(-st*t.x - ct*n.x);
    out->s2.p1.y = o1->c.y + o1->r*(-st*t.y - ct*n.y);
        out_s->s1.p2 = out->s2.p1;

    out->s1.p2.x = o2->c.x + o2->r*(-st*t.x + ct*n.x);
    out->s1.p2.y = o2->c.y + o2->r*(-st*t.y + ct*n.y);
        out_s->s2.p1 = out->s1.p2;

    out->s2.p2.x = o2->c.x + o2->r*(-st*t.x - ct*n.x);
    out->s2.p2.y = o2->c.y + o2->r*(-st*t.y - ct*n.y);
        out_s->s1.p1 = out->s2.p2;

    if(out->d < o1->r + o2->r + 4*LOW_THR) {
        return 2;
    }
    else if(o1->r < LOW_THR) {
        out->s3 = out->s1;
        out->s4 = out->s2;
            out_s->s4 = out_s->s1;
            out_s->s3 = out_s->s2;

        return 2;
    }
    else if(o2->r < LOW_THR) {
        out->s4 = out->s1;
        out->s3 = out->s2;
            out_s->s3 = out_s->s1;
            out_s->s4 = out_s->s2;

        return 2;
    }
    else {
        // 2 internal tangents
        st = (o1->r + o2->r)/out->d;
        ct = sqrt(1 - st*st);

        out->s3.p1.x = o1->c.x - o1->r*(-st*t.x + ct*n.x);
        out->s3.p1.y = o1->c.y - o1->r*(-st*t.y + ct*n.y);
            out_s->s3.p2 = out->s3.p1;

        out->s4.p1.x = o1->c.x - o1->r*(-st*t.x - ct*n.x);
        out->s4.p1.y = o1->c.y - o1->r*(-st*t.y - ct*n.y);
            out_s->s4.p2 = out->s4.p1;

        out->s3.p2.x = o2->c.x + o2->r*(-st*t.x + ct*n.x);
        out->s3.p2.y = o2->c.y + o2->r*(-st*t.y + ct*n.y);
            out_s->s3.p1 = out->s3.p2;

        out->s4.p2.x = o2->c.x + o2->r*(-st*t.x - ct*n.x);
        out->s4.p2.y = o2->c.y + o2->r*(-st*t.y - ct*n.y);
            out_s->s4.p1 = out->s4.p2;

        return 4;
    }

    return 0;   // makes the compiler happy
}

uint8_t check_segment(iObs_t o1, sSeg_t *s, iObs_t o2) {
    iObs_t i;
    sNum_t d;

#ifdef CHECK_LIMITS
    if(OUT(s->p1.x, s->p1.y) || OUT(s->p2.x, s->p2.y))
        return 0;
#endif

    for(i = 0; i < N; i++) {
        if(i == o1 || i == o2 || obs[i].r < LOW_THR || !obs[i].active)
            continue;

        sqdistPt2Seg(&obs[i].c, s, &d, NULL);

        if(d < (obs[i].r + LOW_THR)*(obs[i].r + LOW_THR))
            return 0;
    }

    return 1;
}

//#define FILL_DEBUG
void fill_tgts_lnk() {
    iObs_t i, j;
    uint8_t ok, nb;

    for(i=0; i<N; i++) {
        for(j=i+1; j<N; j++) {
#ifdef FILL_DEBUG
printf("step: obstacles %u and %u\n", i, j);
#endif

            nb = fill_tgts(i, j);

#ifdef FILL_DEBUG
printf("  %u common tangents\n", nb);
printf("  dist %.2f\n", DIST(i, j));
#endif
            switch(nb) {
            case 4:
                ok = check_segment(i, &tgts[i][j].s4, j);

                aselts[A(i)][B(j)].active = ok;
                aselts[A(j)][B(i)].active = ok;
                /* no break */
            case 3:
                ok = check_segment(i, &tgts[i][j].s3, j);

                aselts[B(i)][A(j)].active = ok;
                aselts[B(j)][A(i)].active = ok;

                if(nb == 3) {
                    aselts[A(i)][B(j)].active = 0;
                    aselts[A(j)][B(i)].active = 0;
                }
                /* no break */
            case 2:
                ok = check_segment(i, &tgts[i][j].s2, j);

                if(nb == 2) {
                    aselts[A(i)][B(j)].active = 0;
                    aselts[A(j)][B(i)].active = 0;

                    aselts[B(i)][A(j)].active = 0;
                    aselts[B(j)][A(i)].active = 0;
                }

                if(nb == 2 && obs[i].r < LOW_THR) {    // point/circle case
                    aselts[A(i)][B(j)].active = ok;
                    aselts[A(j)][A(i)].active = ok;
                }
                else if(nb == 2 && obs[j].r < LOW_THR) {    // point/circle case
                    aselts[B(i)][A(j)].active = ok;
                    aselts[A(j)][A(i)].active = ok;
                }
                else {
                    aselts[B(i)][B(j)].active = ok;
                    aselts[A(j)][A(i)].active = ok;
                }
                /* no break */
            case 1:
                ok = check_segment(i, &tgts[i][j].s1, j);

                if(nb == 1) {
                    aselts[A(i)][B(j)].active = 0;
                    aselts[A(j)][B(i)].active = 0;

                    aselts[B(i)][A(j)].active = 0;
                    aselts[B(j)][A(i)].active = 0;

                    aselts[B(i)][B(j)].active = 0;
                    aselts[A(j)][A(i)].active = 0;
                }

                if(nb == 2 && obs[i].r < LOW_THR) {    // point/circle case
                    aselts[A(i)][A(j)].active = ok;
                    aselts[B(j)][A(i)].active = ok;
                }
                else if(nb == 2 && obs[j].r < LOW_THR) {    // point/circle case
                    aselts[A(i)][A(j)].active = ok;
                    aselts[A(j)][B(i)].active = ok;
                }
                else if(nb == 1) {  // point/point case
                    aselts[A(i)][A(j)].active = ok;
                    aselts[A(j)][A(i)].active = ok;
                }
                else {
                    aselts[A(i)][A(j)].active = ok;
                    aselts[B(j)][B(i)].active = ok;
                }
                break;
            default:
            case 0:
                aselts[A(i)][B(j)].active = 0;
                aselts[A(j)][B(i)].active = 0;

                aselts[B(i)][A(j)].active = 0;
                aselts[B(j)][A(i)].active = 0;

                aselts[B(i)][B(j)].active = 0;
                aselts[A(j)][A(i)].active = 0;

                aselts[A(i)][A(j)].active = 0;
                aselts[B(j)][B(i)].active = 0;
                break;
            }
        }
    }
}

uint8_t o_check_arc(iObs_t o1, sPt_t *p2_1, iObs_t o2, int dir, sPt_t *p2_3, iObs_t o3) {
    iObs_t i;
    sVec_t v1, v3;
    sLin_t l1, l3;
    sNum_t sc1, sc3, cross;

    // calc equations of the 2 lines
    convPts2Vec(&obs[o2].c, p2_1, &v1);
    convVecPt2Line(&v1, &obs[o2].c, 0, &l1);

    convPts2Vec(&obs[o2].c, p2_3, &v3);
    convVecPt2Line(&v3, &obs[o2].c, 0, &l3);

    crossVecs(&v1, &v3, &cross);

    // TODO check limits

    if(!dir) {  // clock wise
        for(i = 0; i < N; i++) {
            if(i == o1 || i == o2 || i == o3 || obs[i].r < LOW_THR || !obs[i].active)
                continue;

            sc1 = l1.a*obs[i].c.x + l1.b*obs[i].c.y + l1.c;
            sc3 = l3.a*obs[i].c.x + l3.b*obs[i].c.y + l3.c;

            if(cross < 0) {
                if(sc1 <= 0 || sc3 >= 0)
                    continue;
            }
            else {
                if(sc1 <= 0 && sc3 >= 0)
                    continue;
            }

            if(DIST(i, o2) < obs[i].r + obs[o2].r + LOW_THR)
                return 0; // bad arc
        }
    }
    else {  // counter clock wise
        for(i = 0; i < N; i++) {
            if(i == o1 || i == o2 || i == o3 || obs[i].r < LOW_THR || !obs[i].active)
                continue;

            sc1 = l1.a*obs[i].c.x + l1.b*obs[i].c.y + l1.c;
            sc3 = l3.a*obs[i].c.x + l3.b*obs[i].c.y + l3.c;

            if(cross < 0) {
                if(sc1 >= 0 && sc3 <= 0)
                    continue;
            }
            else {
                if(sc1 >= 0 || sc3 <= 0)
                    continue;
            }

            if(DIST(i, o2) < obs[i].r + obs[o2].r + LOW_THR)
                return 0; // bad arc
        }
    }

    return 1; // good arc
}

sNum_t o_arc_len(sPt_t *p2_1, iObs_t o2, int dir, sPt_t *p2_3) {
    sVec_t v1, v3;
    sNum_t d, c;

    if(obs[o2].r < LOW_THR)
        return 0.;

    convPts2Vec(&obs[o2].c, p2_1, &v1);
    convPts2Vec(&obs[o2].c, p2_3, &v3);

    dotVecs(&v1, &v3, &d);
    crossVecs(&v1, &v3, &c);

    d = acos(d/(obs[o2].r*obs[o2].r));

    if(!dir) {  // clock wise
        if(c > 0) {
            d += M_PI;
        }
    }
    else {  // counter clock wise
        if(c < 0) {
            d += M_PI;
        }
    }

    return d*obs[o2].r;
}
