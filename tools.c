#include <math.h>
#include "math_ops.h"

#include "tools.h"

#define A(i) ((i)<<1)
#define B(i) (((i)<<1)+1)

// array of physical obstacles (256B)
sObs_t obs[N] = {
    {{10., 10.}, 0.},  // start point (current position)
    {{90., 20.}, 20.},
    {{110., 60.}, 20.},
//    {{120., 30.}, 10.},
//    {{110., 40.}, 15.},
    {{210., 10.}, 0.}, // end point (goal)
    {{110., 83.}, 5.}   // check_arc test
};
// tangents between physical obstacles (17kiB)
sTgts_t tgts[N][N] = {{{{{0}}}}};
// halfmatrix of 2Nx2N links between logical obstacles (1kiB)
sLnk_t lnk[2*N][2*N] = {{0}};

uint8_t get_tangents(uint8_t _o1, uint8_t _o2) {
    sVec_t o1o2, t, n;
    sNum_t st, ct;
    uint8_t nb;
    sTgts_t *out = &tgts[_o1][_o2], *out_s = &tgts[_o2][_o1];
    sObs_t *o1 = &obs[_o1];
    sObs_t *o2 = &obs[_o2];

    convPts2Vec(&o1->c, &o2->c, &o1o2);

    normVec(&o1o2, &out->d);
        out_s->d = out->d;

    if(out->d < 2*LOW_THR) {
        return 0;
    }
    else if(o1->r < LOW_THR && o2->r < LOW_THR) {
        out->s1.p1 = o1->c;
        out->s1.p2 = o2->c;
            out_s->s1.p1 = out->s1.p2;
            out_s->s1.p2 = out->s1.p1;

        return 1;
    }
    else if(o1->r + out->d < o2->r + 4*LOW_THR || o2->r + out->d < o1->r + 4*LOW_THR) {
        return 0;
    }

    t.x = o1o2.x/out->d;
    t.y = o1o2.y/out->d;

    n.x = -t.y;
    n.y = t.x;

    if(out->d < o1->r + o2->r + 4*LOW_THR || o1->r < LOW_THR || o2->r < LOW_THR) {
        nb = 2;
    }
    else {
        nb = 4;

        // 2 internal tangents
        st = (o1->r + o2->r)/out->d;
        ct = sqrt(1 - st*st);

        out->s3.p1.x = o1->c.x - o1->r*(-st*t.x + ct*n.x);
        out->s3.p1.y = o1->c.y - o1->r*(-st*t.y + ct*n.y);
            out_s->s4.p2 = out->s3.p1;

        out->s4.p1.x = o1->c.x - o1->r*(-st*t.x - ct*n.x);
        out->s4.p1.y = o1->c.y - o1->r*(-st*t.y - ct*n.y);
            out_s->s3.p2 = out->s4.p1;

        out->s3.p2.x = o2->c.x + o2->r*(-st*t.x + ct*n.x);
        out->s3.p2.y = o2->c.y + o2->r*(-st*t.y + ct*n.y);
            out_s->s4.p1 = out->s3.p2;

        out->s4.p2.x = o2->c.x + o2->r*(-st*t.x - ct*n.x);
        out->s4.p2.y = o2->c.y + o2->r*(-st*t.y - ct*n.y);
            out_s->s3.p1 = out->s4.p2;
    }

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

    return nb;
}

uint8_t check_segment(uint8_t o1, uint8_t o2, sSeg_t *s) {
    int i;
    sNum_t d;

    for(i = 0; i < N; i++) {
        if(i == o1 || i == o2 || obs[i].r < LOW_THR)
            continue;

        sqdistPt2Seg(&obs[i].c, s, &d, NULL);

        if(d < (obs[i].r + LOW_THR)*(obs[i].r + LOW_THR))
            return 0;
    }

    return 1;
}

void get_links() {
    uint8_t i, j, ok, nb;

    for(i=0; i<N; i++) {
        for(j=i+1; j<N; j++) {
            printf("step: obstacles %u and %u\n", i, j);

            nb = get_tangents(i, j);

            printf("  %u common tangents\n", nb);
            printf("  dist %.2f\n", tgts[i][j].d);

            switch(nb) {
            case 4:
                ok = check_segment(i, j, &tgts[i][j].s4);

                lnk[A(i)][B(j)] = ok;
                lnk[A(j)][B(i)] = ok;
            case 3:
                ok = check_segment(i, j, &tgts[i][j].s3);

                lnk[B(i)][A(j)] = ok;
                lnk[B(j)][A(i)] = ok;
            case 2:
                ok = check_segment(i, j, &tgts[i][j].s2);

                if(nb == 2 && obs[i].r < LOW_THR) {    // point/circle case
                    lnk[A(i)][B(j)] = ok;
                    lnk[A(j)][A(i)] = ok;
                }
                else if(nb == 2 && obs[j].r < LOW_THR) {    // point/circle case
                    lnk[B(i)][A(j)] = ok;
                    lnk[A(j)][A(i)] = ok;
                }
                else {
                    lnk[B(i)][B(j)] = ok;
                    lnk[A(j)][A(i)] = ok;
                }
            case 1:
                ok = check_segment(i, j, &tgts[i][j].s1);

                if(nb == 2 && obs[i].r < LOW_THR) {    // point/circle case
                    lnk[A(i)][A(j)] = ok;
                    lnk[B(j)][A(i)] = ok;
                }
                else if(nb == 2 && obs[j].r < LOW_THR) {    // point/circle case
                    lnk[A(i)][A(j)] = ok;
                    lnk[A(j)][B(i)] = ok;
                }
                else {
                    lnk[A(i)][A(j)] = ok;
                    lnk[B(j)][B(i)] = ok;
                }
            default:
            case 0:
                break;
            }
        }
    }
}

uint8_t check_arc(uint8_t o1, sPt_t *p1, uint8_t o2, uint8_t o3, sPt_t *p3, uint8_t dir) {
    int i;
    sVec_t v1, v3;
    sLin_t l1, l3;
    sNum_t sc1, sc3, cross;

    convPts2Vec(&obs[o2].c, p1, &v1);
    convVecPt2Line(&v1, &obs[o2].c, 0, &l1);

    convPts2Vec(&obs[o2].c, p3, &v3);
    convVecPt2Line(&v3, &obs[o2].c, 0, &l3);

    crossVecs(&v1, &v3, &cross);

    if(!dir) {  // clock wise
        for(i = 0; i < N; i++) {
            if(i == o1 || i == o2 || i == o3 || obs[i].r < LOW_THR)
                continue;

            sc1 = l1.a*obs[i].c.x + l1.b*obs[i].c.y + l1.c;
            sc3 = l3.a*obs[i].c.x + l3.b*obs[i].c.y + l3.c;

            if(cross < 0) {
                if(sc1 < 0 || sc3 > 0)
                    continue;
            }
            else {
                if(sc1 < 0 && sc3 > 0)
                    continue;
            }

            if(tgts[i][o2].d < obs[i].r + obs[o2].r + LOW_THR)
                return 0;
        }
    }
    else {  // counter clock wise
        for(i = 0; i < N; i++) {
            if(i == o1 || i == o2 || i == o3 || obs[i].r < LOW_THR)
                continue;

            sc1 = l1.a*obs[i].c.x + l1.b*obs[i].c.y + l1.c;
            sc3 = l3.a*obs[i].c.x + l3.b*obs[i].c.y + l3.c;

            if(cross < 0) {
                if(sc1 > 0 && sc3 < 0)
                    continue;
            }
            else {
                if(sc1 > 0 || sc3 < 0)
                    continue;
            }

            if(tgts[i][o2].d < obs[i].r + obs[o2].r + LOW_THR)
                return 0;
        }
    }

    return 1;
}

sNum_t arc_len(sPt_t *p1, uint8_t o2, sPt_t *p3, uint8_t dir) {
    sVec_t v1, v3;
    sNum_t d, c, t;

    convPts2Vec(&obs[o2].c, p1, &v1);
    convPts2Vec(&obs[o2].c, p3, &v3);

    dotVecs(&v1, &v3, &d);
    crossVecs(&v1, &v3, &c);

    t = acos(d/(obs[o2].r*obs[o2].r));

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

    return t*obs[o2].r;
}

