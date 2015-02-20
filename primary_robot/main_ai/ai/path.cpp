/*
 * path.cpp
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#include <path.h>

#include <cmath>
#include <iostream>
#include "tools.h"
#include "main.h"
#include "math_ops.h"

#include "obj_statuses.h"

extern "C"{
#include "roles.h"
#include <unistd.h>
}


Path::Path() : _dist(0), _path_len(0){
}

Path::Path(vector <sTrajEl_t*> list) : _dist(0), _path_len(list.size()), _path(list){
}

Path::~Path() {
    // TODO Auto-generated destructor stub
}

void Path::clear(){
    _dist = 0;
    _path_len = 0 ;

    //for(sTrajEl_t* i : _path){
    for(unsigned int i ; i < _path.size() ; i++){
        delete _path[i];
    }

    _path.clear();
}


/*
 * Send the path to the robot.
 * Every parameter that can be calculated automatically, the calculation is performed.
 * Try to send MAX_RETRIES if failed.
 */
void Path::sendRobot() {
    sMsg outMsg = { { 0 } };
    int ret;
    static unsigned int tid = 0;

    length(); //compute _dist, arc_len and seg_len //TODO check length between compute and save to the path.
    _path_len = _path.size();

    tid++;

    if (!_path.empty()){
        for (unsigned int i = 0; i < _path_len; i++) {
            printElTraj(i);

            outMsg.header.type = E_TRAJ;
            outMsg.header.size = sizeof(outMsg.payload.traj);

            outMsg.payload.traj.p1_x = _path[i]->p1.x;
            outMsg.payload.traj.p1_y = _path[i]->p1.y;
            outMsg.payload.traj.p2_x = _path[i]->p2.x;
            outMsg.payload.traj.p2_y = _path[i]->p2.y;
            outMsg.payload.traj.seg_len = _path[i]->seg_len;

            outMsg.payload.traj.c_x = _path[i]->obs.c.x;
            outMsg.payload.traj.c_y = _path[i]->obs.c.y;
            outMsg.payload.traj.c_r = _path[i]->obs.r;
            outMsg.payload.traj.arc_len = _path[i]->arc_len;

            outMsg.payload.traj.sid = i;
            outMsg.payload.traj.tid = tid;

            if ((ret = role_sendRetry(&outMsg, MAX_RETRIES)) <= 0) {
                printf("Error [path.cpp] : role_sendRetry(E_TRAJ) failed #%i\n", -ret);
            }

            usleep(1000);
        }
    }
}

void Path::stopRobot() {
    sTrajEl_t *traj = new sTrajEl_t{ { obs[0].c.x, obs[0].c.y }, { obs[0].c.x, obs[0].c.y }, { { obs[0].c.x, obs[0].c.y }, 0, 0, 1 }, 0, 0, 0 }; //TODO used get generic status

    clear();

    _path.push_back(traj);

    sendRobot();
}


/*
 * The robot go to the destination point.
 * "f" to force the robot to go, even if the destination point is in obstacle.
 */
void Path::go2Point(const sPt_t &robot, const sPt_t &dest, const bool f){
    clear();

    sTrajEl_t *traj = new sTrajEl_t;

    traj->p1 = robot; //TODO used get generic status
    traj->p2 = dest;
    traj->obs = {{0., 0.}, 0. ,0 ,0 ,0};

    _path.push_back(traj);

    sendRobot();
}


void Path::followPath(vector <sObs_t> &_obs, vector <iABObs_t> &l) { // todo tableau statique
    clear();


   //copier _obs dans obs

    for (unsigned int i = 0; i < l.size()-1; i++) {
        sTrajEl_t* el = new sTrajEl_t;

        sSeg_t *s = tgt(l[i], l[i + 1]);

        el->p1 = s->p1;
        el->p2 = s->p2;
        el->obs.active = 1;
        el->obs.c = obs[O(l[i + 1])].c;
        el->obs.moved = 1;
        el->obs.r = fabs(obs[O(l[i + 1])].r) * (1 - 2 * DIR(l[i + 1]));

        _path.push_back(el);
    }

    sendRobot();
}

/*
 * Print to the display the trajectory element choose.
 */
void const Path::printElTraj(const unsigned int num){

    if(_path.size() > num)
        cout << "El " << num << " : p1 x" << _path[num]->p1.x << " p1 y" << _path[num]->p1.y
                             << " ; p2 x" << _path[num]->p2.x << " p2 y" << _path[num]->p2.y
                             << " ; obs x" << _path[num]->obs.c.x << " y" << _path[num]->obs.c.y << " r" << _path[num]->obs.r
                             << " ; a_l" << _path[num]->arc_len << " s_l" << _path[num]->seg_len
                             << endl;
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



/*
 * Return the total length path
 */
sNum_t Path::length(){

    setPathLength();

    for (unsigned int i = 0; i < _path_len; i++) {
        _dist += _path[i]->seg_len + _path[i]->arc_len;
    }
    return _dist;
}


/*
 * Compute the length of each path elements and set the total length of the final path from the list of trajectories elements.
 * Be careful, the path must be respect the standards. Especially, the arc_len of the last path element should be null.
 */
void Path::setPathLength() {

    for (unsigned int i = 0; i < _path.size() - 1; i++) {
        _path[i]->arc_len = arc_len2(&(_path[i]->p2), &(_path[i]->obs.c), _path[i]->obs.r, &(_path[i+1]->p1));
        distPt2Pt(&_path[i]->p1, &_path[i]->p2, &_path[i]->seg_len );
    }

    _path[_path.size() - 1]->arc_len = 0;
    distPt2Pt(&_path[_path.size() - 1]->p1, &_path[_path.size() - 1]->p2, &_path[_path.size() - 1]->seg_len);
}
