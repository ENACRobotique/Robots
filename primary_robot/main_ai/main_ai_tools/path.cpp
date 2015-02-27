/*
 * path.cpp
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#include "path.h"

#include <astar_tools.h>
#include <main_ai_tools/path.h>
#include <main_ai_tools/statuses.h>
#include <tools.h>
#include <cmath>
#include <iostream>
#include "math_ops.h"
#include "a_star.h"
#include "time_tools.h"

extern "C"{
#include "roles.h"
#include "millis.h"
#include <unistd.h>
}

Path::Path() : _dist(0), _path_len(0){
}

Path::Path(vector <sTrajEl_t*> list) : _dist(0), _path_len(list.size()){
    //FIXME _path list
}

Path::~Path() {
    // TODO Auto-generated destructor stub
}

void Path::clear(){
    _dist = 0;
    _path_len = 0 ;

    _path.clear();
}

void Path::maintenace(){
    checkRobotBlock();
}


/*
 * Send the path to the robot.
 * Every parameter that can be calculated automatically, the calculation is performed.
 * Try to send MAX_RETRIES if failed.
 */
void Path::sendRobot() {
    sMsg outMsg = { { 0 } };
    static sPath_t path;
    int ret;

    length(); //compute _dist, arc_len and seg_len //TODO check length between compute and save to the path.
    _path_len = _path.size();

    cout << "[INFO] Try to send a path" << endl;

    if (!checkSamePath(path) || checkRobotBlock()){
        if (!_path.empty()){
            net.sendPath(_path);
        }
    }
}

void Path::stopRobot() {
    sTrajEl_t *traj = new sTrajEl_t{ { obs[0].c.x, obs[0].c.y }, { obs[0].c.x, obs[0].c.y }, { { obs[0].c.x, obs[0].c.y }, 0, 0, 1 }, 0, 0, 0 }; //TODO used get generic status

    clear();

    _path.push_back(*traj);

    sendRobot();
}


/*
 * The robot go to the destination point.
 * "f" to force the robot to go, even if the destination point is in obstacle.
 */
void Path::go2Point(const sPt_t &dest, const bool f){
#if 0
    clear();

    sTrajEl_t *traj = new sTrajEl_t;

    traj->p1 = robot;
    traj->p2 = dest;
    traj->obs = {{0., 0.}, 0. ,0 ,0 ,0};

    _path.push_back(traj);
#endif

    obs[0].c = statuses.getLastPosXY(ELT_PRIMARY);
    obs[N-1].c = dest;

    sNum_t dist;
    distPt2Pt(&obs[0].c, &obs[N-1].c, &dist);
    if(dist < 2.){
        return;
    }

    fill_tgts_lnk();

    sPath_t path;
    a_star(A(0), A(N-1), &path);
    if (path.path) {
        printf("new path from 0a to %ua (%.2fcm, %u steps):\n", N - 1, path.dist, path.path_len);

        addPath2(path);
    }
    else {
        printf("no path from 0a to %ua\n", N - 1);
    }

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

        _path.push_back(*el);
    }

    sendRobot();
}

/*
 * Print to the display the trajectory element choose.
 */
void const Path::printElTraj(const unsigned int num){

    if(_path.size() > num)
        cout << "El " << num << " : p1 x" << _path[num].p1.x << " p1 y" << _path[num].p1.y
                             << " ; p2 x" << _path[num].p2.x << " p2 y" << _path[num].p2.y
                             << " ; obs x" << _path[num].obs.c.x << " y" << _path[num].obs.c.y << " r" << _path[num].obs.r
                             << " ; a_l" << _path[num].arc_len << " s_l" << _path[num].seg_len
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

    _dist = 0;

    for (unsigned int i = 0; i < _path.size(); i++) {
        _dist += _path[i].seg_len + _path[i].arc_len;
    }
    return _dist;
}

void Path::addPath(vector <sTrajEl_t*> list){
    clear();

    for(unsigned int i = 0 ; i < list.size() ; i++){
        _path.push_back(*list[i]);
    }
}

void Path::addPath2(sPath_t path){
    clear();

    for(unsigned int i = 0 ; i < path.path_len ; i++){
        _path.push_back(path.path[i]);
    }
}



/*
 * Compute the length of each path elements and set the total length of the final path from the list of trajectories elements.
 * Be careful, the path must be respect the standards. Especially, the arc_len of the last path element should be null.
 */
void Path::setPathLength() {

    if(!_path.empty()){
        for (unsigned int i = 0; i < _path.size() - 1; i++) {
            _path[i].arc_len = arc_len2(&(_path[i].p2), &(_path[i].obs.c), _path[i].obs.r, &(_path[i+1].p1));
            distPt2Pt(&_path[i].p1, &_path[i].p2, &_path[i].seg_len );
        }

        _path[_path.size() - 1].arc_len = 0;
        distPt2Pt(&_path[_path.size() - 1].p1, &_path[_path.size() - 1].p2, &_path[_path.size() - 1].seg_len);
    }
}


int Path::same_obs (sObs_t *obs1, sObs_t *obs2){
    if(verbose > 2)
        printf("r1=%f r2=%f\n",obs1->r,obs2->r);
    return ( obs1->r == obs2->r && obs1->c.x == obs2->c.x && obs1->c.y == obs2->c.y);
    }

/*
 * Return 1 if the same path else 0.
 */
int Path::checkSamePath(sPath_t &path) {
    unsigned int t1_ind = path.path_len;
    unsigned int t2_ind = _path_len;

    cout << "[INFO] Check if the same path" << endl;

    if(path.path_len==0 || _path_len==0)
        return 0;
    if(verbose > 2)
        cout << "same_t 1.0" << endl;

    while ((int)t1_ind > 0 &&  (int)t2_ind > 0) { //pb si un step est terminer
        if(verbose > 2)
            cout << "same_t 2.0" << endl;
        if (same_obs(&(path.path[t1_ind-1].obs), &(_path[t2_ind-1].obs)) ){
          t1_ind--;
           t2_ind--;
           }
       else return 0;
    }
    if(verbose > 2)
        cout << "same_t 3.0" << endl;
    if (!(same_obs (&(path.path[t1_ind].obs), &(_path[t2_ind].obs))))
        return 0;

    if ( (fabs(path.path[t1_ind].p2.x - _path[t2_ind].p2.x ) > 2.) && (fabs(path.path[t1_ind].p2.y - _path[t2_ind].p2.y) > 2.) )
        return 0;
    else
        if(verbose > 2)
            cout << "same_t 4.0" << endl;

    cout << "[INFO] Same path" << endl;
    return 1 ;
}

/*
 * Return 1 if the robot is block else 0.
 */
int Path::checkRobotBlock() {
    static sPt_t pos[10] = { { 0., 0. } };
    static int pt = 0;
    static unsigned int lastTime = 0;
    int i, cpt = 0;
    sNum_t dist;
    sPt_t pos_robot = statuses.getLastPosXY(ELT_PRIMARY);

    if (fabs(time_diff(millis(), lastTime)) > 200) {
        pos[pt] = pos_robot;
        pt++;
        pt = pt % 10;
        for (i = 0; i < 10; i++) {
            distPt2Pt(&pos_robot, &pos[i], &dist);
            if (dist < 1.)
                cpt++;
        }
        if (cpt >= 10) {
            cout << "[INFO] Robot is blocked" << endl;
            return 1;
        }
        lastTime = millis();
    }

    return 0;
}
