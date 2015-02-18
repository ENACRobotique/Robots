/*
 * path.cpp
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#include <path.h>

#include <cmath>
#include "tools.h"
#include "main.h"
#include "math_ops.h"

#include "obj_statuses.h"

extern "C"{
#include "roles.h"
#include <unistd.h>

}

Path::Path() {
    // TODO Auto-generated constructor stub

}

Path::~Path() {
    // TODO Auto-generated destructor stub
}

/*
 * Send the path to the robot.
 * Try to send MAX_RETRIES if failed.
 */

void Path::sendRobot() {
    sMsg outMsg = { { 0 } };
    int ret;

    last_tid++;

    if (_path.path)
        for (unsigned int i = 0; i < _path.path_len; i++) {
            printf("  %u: p1 x%f y%f, p2 x%f y%f, obs x%f y%f r%.2f, a_l%f s_l%f\n", i, _path.path[i].p1.x, _path.path[i].p1.y, _path.path[i].p2.x, _path.path[i].p2.y, _path.path[i].obs.c.x, _path.path[i].obs.c.y, _path.path[i].obs.r, _path.path[i].arc_len, _path.path[i].seg_len);

            outMsg.header.type = E_TRAJ;
            outMsg.header.size = sizeof(outMsg.payload.traj);

            outMsg.payload.traj.p1_x = _path.path[i].p1.x;
            outMsg.payload.traj.p1_y = _path.path[i].p1.y;
            outMsg.payload.traj.p2_x = _path.path[i].p2.x;
            outMsg.payload.traj.p2_y = _path.path[i].p2.y;
            outMsg.payload.traj.seg_len = _path.path[i].seg_len;

            outMsg.payload.traj.c_x = _path.path[i].obs.c.x;
            outMsg.payload.traj.c_y = _path.path[i].obs.c.y;
            outMsg.payload.traj.c_r = _path.path[i].obs.r;
            outMsg.payload.traj.arc_len = _path.path[i].arc_len;

            outMsg.payload.traj.sid = i;
            outMsg.payload.traj.tid = last_tid;

            if ((ret = role_sendRetry(&outMsg, MAX_RETRIES)) <= 0) {
                printf("role_sendRetry(E_TRAJ) failed #%i\n", -ret);
            }

            usleep(1000);
        }
}

void Path::stopRobot() {
    sTrajEl_t traj = { { obs[0].c.x, obs[0].c.y }, { obs[0].c.x, obs[0].c.y }, { { obs[0].c.x, obs[0].c.y }, 0, 0, 1 }, 0, 0, 0 };

    _path.path = &traj;
    _path.path_len = 1;

    sendRobot();
}

int Path::sendSeg(const sPt_t *p, const sVec_t *v) { //the robot goes directly to the point or the vector
    sPath_t path;
    sTrajEl_t t = { { 0 } };

    if (((p == NULL) && (v == NULL)) || ((p != NULL) && (v != NULL))) {
        return -1;
    }

    if (p != NULL) {
        t.p1 = obs[0].c;
        t.p2 = *p;
        distPt2Pt(&t.p1, &t.p2, &t.seg_len);

        t.obs.c = t.p2;
        t.obs.r = 0.;
        t.arc_len = 0.;
        t.sid = 0;
    }

    if (v != NULL) {
        t.p1 = obs[0].c;
        t.p2.x = t.p1.x + v->x;
        t.p2.y = t.p1.y + v->y;
        distPt2Pt(&t.p1, &t.p2, &t.seg_len);

        t.obs.c = t.p2;
        t.obs.r = 0.;
        t.arc_len = 0.;
        t.sid = 0;
    }

    _path.path = &t;
    _path.path_len = 1;
    sendRobot();

    return 1;
}


void followProg(){
    sGenericStatus *stPr = getLastPGStatus(ELT_PRIMARY);
    sPt_t ptPr;
    sGenericStatus *stAPr = getLastPGStatus(ELT_ADV_PRIMARY);
    sPt_t ptAPr;
    sGenericStatus *stASc = getLastPGStatus(ELT_ADV_SECONDARY);
    sPt_t ptASc;
    sNum_t d, dot;
    sVec_t v1, v2;
    int contact = 0;

    if (stPr) {
        ptPr.x = stPr->prop_status.pos.x;
        ptPr.y = stPr->prop_status.pos.y;

        if (stAPr) {
            ptAPr.x = stAPr->prop_status.pos.x;
            ptAPr.y = stAPr->prop_status.pos.y;

            distPt2Pt(&ptPr, &ptAPr, &d);
            v1.x = cos(stPr->prop_status.pos.theta);
            v1.y = sin(stPr->prop_status.pos.theta);
            convPts2Vec(&ptPr, &ptAPr, &v2);
            dotVecs(&v1, &v2, &dot);

            if (d < 50 && dot > 0.6 * d) {
                printf("CONTACT PRIM!!!!!!!!!!!!!!!!!!!!!!!!!\n\n"); // TODO
                contact = 1;
            }
        }

        if (stASc) {
            ptASc.x = stASc->prop_status.pos.x;
            ptASc.y = stASc->prop_status.pos.y;

            distPt2Pt(&ptPr, &ptASc, &d);
            v1.x = cos(stPr->prop_status.pos.theta);
            v1.y = sin(stPr->prop_status.pos.theta);
            convPts2Vec(&ptPr, &ptASc, &v2);
            dotVecs(&v1, &v2, &dot);

            if (d < 40 && dot > 0.6 * d) {
                printf("CONTACT SEC!!!!!!!!!!!!!!!!!!!!!!!!!\n\n"); // TODO
                contact = 1;
            }
        }

        if (contact) {
            sMsg outMsg = { { 0 } };

            outMsg.header.type = E_TRAJ;
            outMsg.header.size = sizeof(outMsg.payload.traj);
            outMsg.payload.traj.p1_x = ptPr.x;
            outMsg.payload.traj.p1_y = ptPr.y;
            outMsg.payload.traj.p2_x = ptPr.x;
            outMsg.payload.traj.p2_y = ptPr.y;
            outMsg.payload.traj.seg_len = 0.;

            outMsg.payload.traj.c_x = ptPr.x;
            outMsg.payload.traj.c_y = ptPr.y;
            outMsg.payload.traj.c_r = 0.;
            outMsg.payload.traj.arc_len = 0.;

            outMsg.payload.traj.sid = 0;
            outMsg.payload.traj.tid = ++last_tid;

            role_sendRetry(&outMsg, MAX_RETRIES);
        }
    }
}

sNum_t seg_len(sPt_t *p1, sPt_t *p2) {
    return sqrt(fabs(p1->x - p2->x) * fabs(p1->x - p2->x) + fabs(p1->y - p2->y) * fabs(p1->y - p2->y));
}

sNum_t arc_len2(sPt_t *p2_1, sPt_t *oc, sNum_t ori, sPt_t *p2_3) {
    sVec_t v1, v3;
    sNum_t d, c;

    if (fabs(ori) < LOW_THR)
        return 0.;

    convPts2Vec(oc, p2_1, &v1);
    convPts2Vec(oc, p2_3, &v3);

    dotVecs(&v1, &v3, &d);
    crossVecs(&v1, &v3, &c);

    d = d / (ori * ori);
    // d must be between -1 and 1 but because we do not use the true length of v1 and v3
    // (we use r instead to avoid some heavy calculations) it may be a little outside of this interval
    // so, let's just be sure we stay in this interval for acos to give a result
    if (d > 1.) {
        d = 1.;
    }
    else
        if (d < -1.) {
            d = -1.;
        }

    d = acos(d);

    if (ori > 0.) {  // clock wise
        if (c > 0) {
            d = 2 * M_PI - d;
        }
    }
    else {  // counter clock wise
        if (c < 0) {
            d = 2 * M_PI - d;
        }
    }

    return fabs(d * ori);
}

void path_len(sTrajEl_t tab[], int size) {  //FIXME last traj with leng seg
    int i;

    for (i = 0; i < size - 1; i++) {
        tab[i].arc_len = arc_len2(&(tab[i].p2), &tab[i].obs.c, tab[i].obs.r, &(tab[i + 1].p1));
        tab[i].seg_len = seg_len(&(tab[i].p1), &(tab[i].p2));
    }
    tab[size - 1].arc_len = 0;
}

sNum_t distPath(sPath_t *path) {
    int i;
    sNum_t dist = 0;
    for (i = 0; i < path->path_len; i++) {
        dist += path->path->seg_len + path->path->arc_len;
    }
    return dist;
}
