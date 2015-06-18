/*
 * obj_geometry_tools.c
 *
 *  Created on: 17 avr. 2014
 *      Author: seb
 */

#include <geometry_tools.h>
#include "math_ops.h"
#include <cmath>

//For a point

void TransPt(sPt_t *traj, int x, int y) {
    traj->x += x;
    traj->y += y;
}

void Rot90Pt(sPt_t *traj) {
    float tempA;

    tempA = traj->x;
    traj->x = -traj->y;
    traj->y = tempA;
}

void SymPt(sPt_t *traj, int x, int y) { // x!=0 symmetry in x in zero and idem for y
    if (y != 0)
        traj->x *= -1;
    if (x != 0)
        traj->y *= -1;
}

//For a element of trajectory

void TransElTraj(sTrajEl_t *traj, int x, int y) {
    //axis x
    traj->p1.x += x;
    traj->p2.x += x;
    traj->obs.c.x += x;

    //axis y
    traj->p1.y += y;
    traj->p2.y += y;
    traj->obs.c.y += y;
}

void SymElTraj(sTrajEl_t *traj, int x, int y) { // x!=0 symmetry in x in zero and idem for y
    if (y != 0) {
        traj->p1.x *= -1;
        traj->p2.x *= -1;
        traj->obs.c.x *= -1;
        traj->obs.r *= -1;
    }

    if (x != 0) {
        traj->p1.y = -traj->p1.y;
        traj->p2.y = -traj->p2.y;
        traj->obs.c.y = -traj->obs.c.y;
        traj->obs.r = -traj->obs.r;
    }
}

void Rot90Traj(sTrajEl_t *traj) {
    float tempA, tempB, tempC;

    tempA = traj->p1.x;
    tempB = traj->p2.x;
    tempC = traj->obs.c.x;

    traj->p1.x = -traj->p1.y;
    traj->p2.x = -traj->p2.y;
    traj->obs.c.x = -traj->obs.c.y;

    traj->p1.y = tempA;
    traj->p2.y = tempB;
    traj->obs.c.y = tempC;
}

//Projection

void projectPoint(sNum_t xp, sNum_t yp, sNum_t rc, sNum_t xc, sNum_t yc, sPt_t *point) {
    sPt_t o_c = { xc, yc };
    sPt_t o_p = { xp, yp };
    sVec_t v;
    sNum_t r = rc, n, d;
    int sg = 0;

    convPts2Vec(&o_c, &o_p, &v);
    normVec(&v, &n);

    d = n - fabs(r);

    if (fabs(d) > 2.)
        printf("!!! far from the circle (%.2fcm)...\n", d);
    sign(v.x * (fabs(r) + 0.1) / n, &sg);
    point->x = o_c.x + v.x * (fabs(r) + 0.1) / n + 0.1 * sg;
    sign(v.y * (fabs(r) + 0.1) / n, &sg);
    point->y = o_c.y + v.y * (fabs(r) + 0.1) / n + 0.1 * sg;
}


void tranOrg(sPt_t *pRef, sPt_t *p1, sPt_t *p2) {
    p2->x = pRef->x + p1->x;
    p2->y = pRef->y + p1->y;
}
