#include "tools.h"

#include <math.h>
#include <stdio.h>
#include "math_ops.h"

extern "C"{
#include "millis.h"
}

#ifdef AS_STATS
extern "C"{
#include "millis.h"
}
#endif

#define CHECK_LIMITS

// array of physical obstacles (256B)
#if PROG_TRAJ
sObs_t obs[]= {
        // robots
        {{0., 0.}, 0., 1, 1, 1},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},
        {{0., 0.}, 0., 0, 0},

        // feux
        {{40. , 90. }, R_ROBOT+7, 1, 1, 1},//12
        {{90. , 40. }, R_ROBOT+7, 1, 1, 1},
        {{90. , 140.}, R_ROBOT+7, 1, 1, 1},
        {{210., 40. }, R_ROBOT+7, 1, 1, 1},
        {{210., 140.}, R_ROBOT+7, 1, 1, 1},
        {{260., 90. }, R_ROBOT+7, 1, 1, 1},

        {{1.  , 120.}, R_ROBOT+2, 1, 1, 1},
        {{130., 1.  }, R_ROBOT+2, 1, 1, 1},
        {{170., 1.  }, R_ROBOT+2, 1, 1, 1},
        {{299., 120.}, R_ROBOT+2, 1, 1, 1},

        {{0., 0.}, R_ROBOT+2, 1, 1, 1},
};

#else
sObs_t obs[] = {
    // robots
    {{0., 0.}, 0., 1, 1},               //primary
    {{0., 0.}, R_ROBOT+12., 1, 1, 1},   //secondary
    {{0., 0.}, R_ROBOT+20., 1, 1, 1},   //primary adv
    {{0., 0.}, R_ROBOT+15., 1, 1, 1},   //secondary adv

    {{100., 100.}, 20., 1, 1, 1},
    {{200., 100.}, 20., 1, 1, 1},

    //Cercles du robot anti-demi-tour
    {{0., 0. }, 0, 0, 0, 1},
    {{0., 0. }, 0, 0, 0, 1},//45
    {{0., 0. }, 0, 0, 0, 1},

    //Cercles d'approches
    {{0., 0. }, 0, 0, 0, 1},//47
    {{0., 0. }, 0, 0, 0, 1},
    {{0., 0. }, 0, 0, 0, 1},

    // arrivée
    {{0. , 0.}, 0, 0, 1, 1} //51
};
#endif
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

    out->type = E_OBSPAIR_UNKNOWN;
        out_s->type = E_OBSPAIR_UNKNOWN;

    if(!o1->active || !o2->active)
        return 0;

    if(out->d < 2*LOW_THR) {
        out->type = E_OBSPAIR_0S_CONCENTRIC;
            out_s->type = E_OBSPAIR_0S_CONCENTRIC;

        return 0;
    }
    else if(o1->r < LOW_THR && o2->r < LOW_THR) {
        out->s1.p1 = o1->c;
        out->s1.p2 = o2->c;
        out->type = E_OBSPAIR_1S_LINE;
            out_s->s1.p1 = out->s1.p2;
            out_s->s1.p2 = out->s1.p1;
            out_s->type = E_OBSPAIR_1S_LINE;

        return 1;
    }
    else if(o1->r + out->d < o2->r + LOW_THR){
        out->type = E_OBSPAIR_0S_SURROUNDED;
            out_s->type = E_OBSPAIR_0S_SURROUNDS;

        return 0;
    }
    else if(o2->r + out->d < o1->r + LOW_THR) {
        out->type = E_OBSPAIR_0S_SURROUNDS;
            out_s->type = E_OBSPAIR_0S_SURROUNDED;

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

    if(o1->r < LOW_THR) {
            out->s3 = out->s1;
            out->s4 = out->s2;
            out->type = E_OBSPAIR_2S_POINT2CIRCLE;
                out_s->s4 = out_s->s1;
                out_s->s3 = out_s->s2;
                out_s->type = E_OBSPAIR_2S_CIRCLE2POINT;

            return 2;
    }
    else if(o2->r < LOW_THR) {
        out->s4 = out->s1;
        out->s3 = out->s2;
        out->type = E_OBSPAIR_2S_CIRCLE2POINT;
            out_s->s3 = out_s->s1;
            out_s->s4 = out_s->s2;
            out_s->type = E_OBSPAIR_2S_POINT2CIRCLE;

        return 2;
    }
    else if(out->d < o1->r + o2->r + LOW_THR) {
        out->type = E_OBSPAIR_2S_INTERSECTION;
            out_s->type = E_OBSPAIR_2S_INTERSECTION;

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

        out->type = E_OBSPAIR_4S_FULL;
            out_s->type = E_OBSPAIR_4S_FULL;

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

        if(d + 2*LOW_THR < obs[i].r*obs[i].r)
            return 0;
    }

    return 1;
}

//#define FILL_DEBUG
void fill_tgts_lnk() {
    iObs_t i, j;
    uint8_t ok, nb;
#ifdef AS_STATS
    unsigned int start_us = micros();
#endif

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

                if(tgts[i][j].type == E_OBSPAIR_2S_POINT2CIRCLE) {
                    aselts[A(i)][B(j)].active = ok;
                    aselts[A(j)][A(i)].active = ok;
                }
                else if(tgts[i][j].type == E_OBSPAIR_2S_CIRCLE2POINT) {
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

                if(tgts[i][j].type == E_OBSPAIR_2S_POINT2CIRCLE) {
                    aselts[A(i)][A(j)].active = ok;
                    aselts[B(j)][A(i)].active = ok;
                }
                else if(tgts[i][j].type == E_OBSPAIR_2S_CIRCLE2POINT) {
                    aselts[A(i)][A(j)].active = ok;
                    aselts[A(j)][B(i)].active = ok;
                }
                else if(tgts[i][j].type == E_OBSPAIR_1S_LINE) {
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

#ifdef AS_STATS
printf("fill_tgts_lnk executed in %uµs\n", micros() - start_us);
#endif
}

#ifdef CHECK_LIMITS
static inline void _southern_point_arc(const sPt_t *p1, const sPt_t *c, sNum_t r, int dir, const sPt_t *p2, const sNum_t *_cross, sPt_t *psouth){
    int qp1, qp2, type;
    sNum_t cross;

    if(_cross){
        cross = *_cross;
    }
    else{
        sVec_t v1, v3;
        convPts2Vec(c, p1, &v1);
        convPts2Vec(c, p2, &v3);
        crossVecs(&v1, &v3, &cross);
    }

    qp1 = QUADRANT(p1->x - c->x, p1->y - c->y);
    qp2 = QUADRANT(p2->x - c->x, p2->y - c->y);

    // preliminary, all cases: (Qi = quadrant of Pi, c = cross, S = south, My = minimum y, X = impossible case => may be replaced by S or My)
    //            __CCW__   __C W__
    //    Q1 Q2   c>0 c<0 | c>0 c<0
    //    0  0    My  S   | S   My
    //    0  1    My  X   | S   X
    //    0  2    My  My  | S   S
    //    0  3    X   S   | X   My
    //    1  0    X   S   | X   My
    //    1  1    My  S   | S   My
    //    1  2    My  X   | S   X
    //    1  3    S   S   | My  My
    //    2  0    S   S   | My  My
    //    2  1    X   S   | X   My
    //    2  2    My  S   | S   My
    //    2  3    S   X   | My  X
    //    3  0    My  X   | S   X
    //    3  0    My  My  | S   S
    //    3  0    X   My  | X   S
    //    3  0    My  S   | S   My
    // with X replaced in order to have some generalizations
    //            __CCW__   __C W__
    //    Q1 Q2   c>0 c<0 | c>0 c<0
    //    0  0    My  S   | S   My
    //    0  1    My  My  | S   S
    //    0  2    My  My  | S   S
    //    0  3    My  S   | S   My
    //    1  0    S   S   | My  My
    //    1  1    My  S   | S   My
    //    1  2    My  S   | S   My
    //    1  3    S   S   | My  My
    //    2  0    S   S   | My  My
    //    2  1    My  S   | S   My
    //    2  2    My  S   | S   My
    //    2  3    S   S   | My  My
    //    3  0    My  S   | S   My
    //    3  0    My  My  | S   S
    //    3  0    My  My  | S   S
    //    3  0    My  S   | S   My
    // generalizations:
    //   * type(Q1, ...) == type(3-Q1, ...)
    //   * type(Q2, ...) == type(3-Q2, ...)
    //   * type(CW, ...) == !type(CCW, ...)

    // 2 symmetries
    if(qp1 > 1){
        qp1 = 3 - qp1;
    }
    if(qp2 > 1){
        qp2 = 3 - qp2;
    }

    if((!qp2 && cross < 0) || (qp1 && !qp2) || (qp1 && cross < 0)){
        type = 0; // south point
    }
    else{
        type = 1; // point with minimum y
    }

    if(!dir) type = !type;

    if(!type){ // south point
        psouth->x = c->x;
        psouth->y = c->y - r;
    }
    else{ // point with minimum y
        if(p1->y < p2->y){
            *psouth = *p1;
        }
        else{
            *psouth = *p2;
        }

    }
}

void southern_point_arc(const sPt_t *p1, iObs_t o, int dir, const sPt_t *p2, const sNum_t *_cross, sPt_t *psouth){
    _southern_point_arc(p1, &obs[o].c, obs[o].r, dir, p2, _cross, psouth);
}

void eastern_point_arc(const sPt_t *_p1, iObs_t o, int dir, const sPt_t *_p2, const sNum_t *_cross, sPt_t *peast){
    sPt_t c, p1, p2, psouth;

    c.x = obs[o].c.y;
    c.y = -obs[o].c.x;

    p1.x = _p1->y;
    p1.y = -_p1->x;

    p2.x = _p2->y;
    p2.y = -_p2->x;

    _southern_point_arc(&p1, &c, obs[o].r, dir, &p2, _cross, &psouth);

    peast->x = -psouth.y;
    peast->y = psouth.x;
}

void western_point_arc(const sPt_t *_p1, iObs_t o, int dir, const sPt_t *_p2, const sNum_t *_cross, sPt_t *pwest){
    sPt_t c, p1, p2, psouth;

    c.x = -obs[o].c.y;
    c.y = obs[o].c.x;

    p1.x = -_p1->y;
    p1.y = _p1->x;

    p2.x = -_p2->y;
    p2.y = _p2->x;

    _southern_point_arc(&p1, &c, obs[o].r, dir, &p2, _cross, &psouth);

    pwest->x = psouth.y;
    pwest->y = -psouth.x;
}


void northern_point_arc(const sPt_t *_p1, iObs_t o, int dir, const sPt_t *_p2, const sNum_t *_cross, sPt_t *pnorth){
    sPt_t c, p1, p2, psouth;

    c.x = -obs[o].c.x;
    c.y = -obs[o].c.y;

    p1.x = -_p1->x;
    p1.y = -_p1->y;

    p2.x = -_p2->x;
    p2.y = -_p2->y;

    _southern_point_arc(&p1, &c, obs[o].r, dir, &p2, _cross, &psouth);

    pnorth->x = -psouth.x;
    pnorth->y = -psouth.y;
}
#endif

uint8_t o_check_arc(iObs_t o1, sPt_t *p2_1, iObs_t o2, int dir, sPt_t *p2_3, iObs_t o3) {
    iObs_t i;
    sVec_t v1, v3;
    sLin_t l1, l3;
    sNum_t sc1, sc3, cross;
#ifdef CHECK_LIMITS
    sPt_t furthest_point;
#endif

    // calc equations of the 2 lines
    convPts2Vec(&obs[o2].c, p2_1, &v1);
    convVecPt2Line(&v1, &obs[o2].c, 0, &l1);

    convPts2Vec(&obs[o2].c, p2_3, &v3);
    convVecPt2Line(&v3, &obs[o2].c, 0, &l3);

    crossVecs(&v1, &v3, &cross);

#ifdef CHECK_LIMITS
    if(obs[o2].c.y - obs[o2].r < Y_MIN){
        southern_point_arc(p2_1, o2, dir, p2_3, &cross, &furthest_point);
        if(furthest_point.y < Y_MIN){
            return 0;
        }
    }

    if(obs[o2].c.x + obs[o2].r > X_MAX){
        eastern_point_arc(p2_1, o2, dir, p2_3, &cross, &furthest_point);
        if(furthest_point.x > X_MAX){
            return 0;
        }
    }

    if(obs[o2].c.x - obs[o2].r < X_MIN){
        western_point_arc(p2_1, o2, dir, p2_3, &cross, &furthest_point);
        if(furthest_point.x < X_MIN){
            return 0;
        }
    }

    if(obs[o2].c.y + obs[o2].r > Y_MAX){
        northern_point_arc(p2_1, o2, dir, p2_3, &cross, &furthest_point);
        if(furthest_point.y > Y_MAX){
            return 0;
        }
    }
#endif

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

            if(tgts[o2][i].type == E_OBSPAIR_2S_INTERSECTION){
#ifdef AS_DEBUG
printf("      because of CW %i:", i);
#endif
                return 0; // bad arc
            }
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

            if(tgts[o2][i].type == E_OBSPAIR_2S_INTERSECTION){
#ifdef AS_DEBUG
printf("      because of CCW %i:", i);
#endif
                return 0; // bad arc
            }
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

#if defined(AS_DEBUG) && AS_DEBUG > 2
dumpPt(p2_1,       "      in ");
dumpPt(&obs[o2].c, "      c  ");
printf(            "      r  =%.2f\n", obs[o2].r);
dumpPt(p2_3,       "      out");
dumpVec(&v1,       "      v1");
dumpVec(&v3,       "      v3");
printf(            "      v1^v3=%.3f ; v1.v3=%.3f\n", c, d);
#endif

    d = d/(obs[o2].r*obs[o2].r);
    // d must be between -1 and 1 but because we do not use the true length of v1 and v3
    // (we use r instead to avoid some heavy calculations) it may be a little outside of this interval
    // so, let's just be sure we stay in this interval for acos to give a result
    if(d > 1.) {
        d = 1.;
    }
    else if(d < -1.) {
        d = -1.;
    }

    d = acos(d);

    if(!dir) {  // clock wise
        if(c > 0) {
            d = 2*M_PI - d;
        }
    }
    else {  // counter clock wise
        if(c < 0) {
            d = 2*M_PI - d;
        }
    }

    return d*obs[o2].r;
}
