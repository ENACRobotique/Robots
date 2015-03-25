/*
 * path.cpp
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#include <a_star_tools.h>
#include "path.h"

#include <main_ai_tools/path.h>
#include <main_ai_tools/statuses.h>
#include <tools.h>
#include <cmath>
#include <iostream>
#include "math_ops.h"
#include "a_star.h"
#include "time_tools.h"
#include "tools.h"
#include "ai_types.h"
#include "obj_tools.h"
#include "GeometryTools.h"

extern "C"{
#include "roles.h"
#include "millis.h"
#include <unistd.h>
#include "bn_intp.h"
}

#ifndef HOLONOMIC
#error "HOLONOMIC must be defined"
#endif

Path::Path() : _dist(0), _path_len(0){
}

Path::Path(vector <sTrajEl_t*> list) : _dist(0), _path_len(list.size()){
    for(sTrajEl_t* i : list)
        _path.push_back(*i);
}

Path::Path(vector <sTrajOrientEl_t*> list) : _dist(0), _path_len(list.size()){
    for(sTrajOrientEl_t* i : list)
        _path_orient.push_back(*i);
}

Path::~Path() {
}

void Path::clear(){
    _dist = 0;
    _path_len = 0 ;
    _path.clear();
    _path_orient.clear();
}

/*
 * Maintenance :
 * - Check if the robot is block
 * - Update circle no half return for non holonomic robot
 */
void Path::maintenace(){
    static unsigned int prevTime = 0;

    if(millis() - prevTime > 200){
        if(checkRobotBlock())
            //sendRobot(); FIXME depuis la position courante
        prevTime = millis();
    }

#if !HOLONOMIC
    static unsigned int prevTime2 = 0;
    if(millis() - prevTime2 > 100){
        updateNoHaftTurn();
        prevTime2 = millis();
    }
#endif
}

/*
 * Send the path to the robot.
 * Every parameter that can be calculated automatically, the calculation is performed.
 * Try to send MAX_RETRIES if failed.
 */
void Path::sendRobot() {
    static sPath_t path;

    if(!_path.empty() && !_path_orient.empty()){
        logs << ERR << "Try to send a path : _path and _path_orient are not empty";
        return;
    }
    if(_path.empty() && _path_orient.empty()){
        logs << ERR << "Try to send a path : _path and _path_orient are empty ";
        return;
    }

    length();

#if HOLONOMIC
    convPathToPathOrient();
#endif

    if (!checkSamePath(path) || checkRobotBlock()){
        logs << INFO << "Preparation to send a path";
        if (!_path.empty()){
            //delete the previous path sent;
            if(path.path)
                delete path.path;

            //save the new path sent
            path.dist = _dist;
            path.path_len = _path.size();
            path.path = new sTrajEl_t[path.path_len];
            for(unsigned int i = 0 ; i < path.path_len ; i++){
                path.path[i] = _path[i];
            }
            //sends the path
            net.sendPath(_path);

            //print the path to the display
            for(unsigned int i = 0 ; i < _path.size() ; i++)
                printElTraj(i);
        }
        else{ //!_path_orient.empty()
            //sends the path //TODO save path and compare if the same
            net.sendPathOrient(_path_orient);

            //print the path to the display
            for(unsigned int i = 0 ; i < _path_orient.size() ; i++)
                printElTrajOrient(i);
        }
    }
}

/*
 * Stop the robot
 */
void Path::stopRobot() {
    sTrajEl_t traj = sTrajEl_t{statuses.getLastPosXY(ELT_PRIMARY), statuses.getLastPosXY(ELT_PRIMARY), {statuses.getLastPosXY(ELT_PRIMARY), 0, 0, 1, 0}, 0, 0, 0 };

    clear();

    _path.push_back(traj);

    sendRobot();
}


/*
 * The robot go to the destination point.
 * "f" to force the robot to go, even if the destination point is in obstacle or if there are obstacles in the trajectory.
 */
void Path::go2Point(const sPt_t &dest, const bool f){
    sPath_t path;

    obs[0].c = statuses.getLastPosXY(ELT_PRIMARY);
    obs[N-1].c = dest;
    logs << DEBUG << "position : " << obs[0].c.x << ", " << obs[0].c.y << " ; destination : " << obs[N-1].c.x << ", " << obs[N-1].c.y;

    Point2D<float> p1(obs[0].c.x, obs[0].c.y),  p2(obs[N-1].c.x, obs[N-1].c.y);
    if(p1.distanceSqTo(p2) < 2.*2.)
        return;

    if(f){
        clear();
        sTrajEl_t traj = sTrajEl_t{statuses.getLastPosXY(ELT_PRIMARY), dest, {dest, 0., 0, 1, 0}, 0, 0, 0 };

        _path.push_back(traj);
        sendRobot();
    }
    else {
        fill_tgts_lnk();
        a_star(A(0), A(N-1), &path);
        if (path.path) {
            logs << INFO << "New path from 0a to " <<  N-1 << " (" << path.dist << ", " << path.path_len << " steps )";
            addPath2(path);
            sendRobot();
        }
        else {
            logs << INFO << "No path from 0a to " << N - 1;
        }
    }
}


void Path::followPath(vector <sObs_t> &_obs, vector <iABObs_t> &l) { // todo tableau statique //for traj prog
    clear();

   //copier _obs dans obs
    //TODO if there are an adversaire

    for (unsigned int i = 0; i < l.size()-1; i++) {
        sTrajEl_t el;

        sSeg_t *s = tgt(l[i], l[i + 1]);

        el.p1 = s->p1;
        el.p2 = s->p2;
        el.obs.active = 1;
        el.obs.c = obs[O(l[i + 1])].c;
        el.obs.moved = 1;
        el.obs.r = fabs(obs[O(l[i + 1])].r) * (1 - 2 * DIR(l[i + 1]));

        _path.push_back(el);
    }

    sendRobot();
}

/*
 * Converts a path in path orient.
 * This path orient can't directly used by a holonomic robot (theta set to 0 and time too)
 */
void Path::convPathToPathOrient(){
    sTrajOrientEl_t trajOrient;

    while(!_path.empty()){
        trajOrient.t1 = 0;
        trajOrient.t2 = 0;

        trajOrient.p1 = _path.front().p1;
        trajOrient.p2 = _path.front().p2;

        trajOrient.obs = _path.front().obs;

        trajOrient.theta1 = 0;
        trajOrient.theta2 = 0;

        trajOrient.seg_len = _path.front().seg_len;
        trajOrient.arc_len = _path.front().arc_len;

        trajOrient.rot1_dir = false;
        trajOrient.rot2_dir = false;

        _path.pop_front();
        _path_orient.push_back(trajOrient);
    }

    computeTimePathForHolonomic();
    computeOrientPathForHolonomic(0); //FIXME
}

/*
 * Computes time path holonomic robot
 */
void Path::computeTimePathForHolonomic(){
    double dist = 0;
    uint32_t time_us = bn_intp_micros2s(micros());

    for(unsigned int i = 0 ; i < _path_orient.size() ; i++){
        _path_orient[i].t1 = (uint32_t)((dist/MAX_SPEED)*1e6) + time_us; //in us
        dist += _path_orient[i].seg_len;
        _path_orient[i].t2 = (uint32_t)((dist/MAX_SPEED)*1e6) + time_us; //in us
        dist += _path_orient[i].arc_len;
    }
}

float getPrincipalAngleValue(float a){
    while(a > M_PI)
        a -= 2*M_PI;
    while(a < -M_PI)
        a += 2*M_PI;
    return a;
}

/*
 * Computes a orient path for holonomic robot
 */
void Path::computeOrientPathForHolonomic(float theta_end_obj){
    //computes theta and rot_dir for all path
    for(unsigned int i = 0 ; i < _path_orient.size() ; i++){
        // in this case, the robot's orientation follows the linear speed vector

        _path_orient[i].theta1 = atan2(_path_orient[i].p2.y - _path_orient[i].p1.y, _path_orient[i].p2.x - _path_orient[i].p1.x);
        _path_orient[i].theta2 = _path_orient[i].theta1;
        _path_orient[i].rot1_dir = true; // unused because theta1 == theta2

        // r>0 CW  => dir=0
        // r<0 CCW => dir=1
        _path_orient[i].rot2_dir = _path_orient[i].obs.r < 0;
    }

    //adjusts for the first element
    float theta = statuses.getLastOrient(ELT_PRIMARY);
//    float diff = fabs(_path_orient.front().theta1 - theta /* XXX you need to get the principal angle value here */)/MAX_SPEED_ROT - _path_orient.front().seg_len/MAX_SPEED;
//    if(diff < 0){
        _path_orient.front().theta1 = theta;
        _path_orient.front().rot1_dir = getPrincipalAngleValue(_path_orient.front().theta2 - _path_orient.front().theta1) > 0; // select shortest rotation
//    }
//    else{ //insert rotation
//        float theta_inter = diff*MAX_SPEED_ROT;
//        sTrajOrientEl_t traj;
//        traj.p1 = statuses.getLastPosXY(ELT_PRIMARY);
//        traj.p2 = traj.p1;
//        traj.obs.c = traj.p1;
//        traj.obs.r = 0;
//        traj.theta1 = statuses.getLastOrient(ELT_PRIMARY);
//        traj.theta2 = theta_inter;
//
//        _path_orient.front().theta1 = theta_inter;
//        _path_orient.push_front(traj);
//    }

    //adjusts for the last element
//    diff = fabs(_path_orient.back().theta1 - theta_end_obj)/MAX_SPEED_ROT - _path_orient.back().seg_len/MAX_SPEED;
//    if(diff < 0){
        _path_orient.back().theta2 = theta_end_obj;
        _path_orient.back().rot1_dir = getPrincipalAngleValue(_path_orient.back().theta2 - _path_orient.back().theta1) > 0; // select shortest rotation
//    }
//    else{
//        float theta_inter = diff*MAX_SPEED_ROT;
//        sTrajOrientEl_t traj;
//        traj.p1 = statuses.getLastPosXY(ELT_PRIMARY);
//        traj.p2 = traj.p1;
//        traj.obs.c = traj.p1;
//        traj.obs.r = 0;
//        traj.theta1 = statuses.getLastOrient(ELT_PRIMARY);
//        traj.theta2 = theta_inter;
//
//        _path_orient.back().theta1 = theta_inter;
//        _path_orient.push_back(traj);
//    }
}

/*
 * Print the trajectory element choose.
 */
void Path::printElTraj(const unsigned int num) const{
    if(_path.size() > num)
        logs << DEBUG << "El " << num
                               << " : p1 x" << _path[num].p1.x << " p1 y" << _path[num].p1.y
                               << " ; p2 x" << _path[num].p2.x << " p2 y" << _path[num].p2.y
                               << " ; obs x" << _path[num].obs.c.x << " y" << _path[num].obs.c.y << " r" << _path[num].obs.r
                               << " ; a_l" << _path[num].arc_len << " s_l" << _path[num].seg_len;
}

void Path::printElTrajOrient(const unsigned int num) const{
    if(_path_orient.size() > num)
        logs << DEBUG << "El " << num
                               << " : t1=" << _path_orient[num].t1 << " t2=" << _path_orient[num].t2
                               << " ; p1 x" << _path_orient[num].p1.x << " p1 y" << _path_orient[num].p1.y
                               << " ; p2 x" << _path_orient[num].p2.x << " p2 y" << _path_orient[num].p2.y
                               << " ; obs x" << _path_orient[num].obs.c.x << " y" << _path_orient[num].obs.c.y << " r" << _path_orient[num].obs.r
                               << " ; theta1=" << _path_orient[num].theta1 << " theta2=" << _path_orient[num].theta2
                               << " ; a_l" << _path_orient[num].arc_len << " s_l" << _path_orient[num].seg_len
                               << " : rot1_dir " << _path_orient[num].rot1_dir << " rot2_dir " << _path_orient[num].rot2_dir;
}

void Path::printPath() const{
    if(!_path.empty() & !_path_orient.empty())
        logs << ERR << "_path and _path_orient are not empty";

    if(!_path.empty())
        for(unsigned int i = 0 ; i < _path.size() ; i++)
            printElTraj(i);
    else if(!_path_orient.empty())
        for(unsigned int i = 0 ; i < _path_orient.size() ; i++)
            printElTrajOrient(i);
}

/*
 * Return the total length path
 * compute _dist, arc_len and seg_len
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
            Circle2D<float> c(_path[i].obs.c.x, _path[i].obs.c.y, _path[i].obs.r);
            Point2D<float> p1(_path[i].p2.x, _path[i].p2.y), p2(_path[i+1].p1.x, _path[i+1].p1.y);
            _path[i].arc_len = c.arcLenght(p1, p2);
            distPt2Pt(&_path[i].p1, &_path[i].p2, &_path[i].seg_len );
        }

        _path[_path.size() - 1].arc_len = 0;
        distPt2Pt(&_path[_path.size() - 1].p1, &_path[_path.size() - 1].p2, &_path[_path.size() - 1].seg_len);
    }
    else if(!_path_orient.empty()){
        for (unsigned int i = 0; i < _path_orient.size() - 1; i++) {
            Circle2D<float> c(_path_orient[i].obs.c.x, _path_orient[i].obs.c.y, _path_orient[i].obs.r);
            Point2D<float> p1(_path_orient[i].p2.x, _path_orient[i].p2.y), p2(_path_orient[i+1].p1.x, _path_orient[i+1].p1.y);
            _path_orient[i].arc_len = c.arcLenght(p1, p2);
            distPt2Pt(&_path_orient[i].p1, &_path_orient[i].p2, &_path_orient[i].seg_len );
        }

        _path_orient[_path_orient.size() - 1].arc_len = 0;
        distPt2Pt(&_path_orient[_path_orient.size() - 1].p1, &_path_orient[_path_orient.size() - 1].p2, &_path_orient[_path_orient.size() - 1].seg_len);
    }
}

/*
 * Return true(1) if the 2 obs are identical else false(0)
 */
bool Path::checkSameObs(sObs_t& obs1, sObs_t& obs2){
    return ( obs1.r == obs2.r && obs1.c.x == obs2.c.x && obs1.c.y == obs2.c.y);
    }

/*
 * Return 1 if the same path else 0.
 */
bool Path::checkSamePath(sPath_t &path){
    unsigned int t1_ind = path.path_len;
    unsigned int t2_ind = _path_len;

    logs << INFO << "Check if the same path";

    if(path.path_len==0 || _path.size()==0)
        return false;
    if(verbose > 2)
        cout << "same_t 1.0" << endl;

    while ((int)t1_ind > 0 &&  (int)t2_ind > 0) { //pb si un step est terminé
        if(verbose > 2)
            cout << "same_t 2.0" << endl;
        if (checkSameObs((path.path[t1_ind-1].obs), (_path[t2_ind-1].obs)) ){
          t1_ind--;
           t2_ind--;
           }
       else return 0;
    }
    if(verbose > 2)
        cout << "same_t 3.0" << endl;
    if (!(checkSameObs((path.path[t1_ind].obs), (_path[t2_ind].obs))))
        return 0;

    if ( (fabs(path.path[t1_ind].p2.x - _path[t2_ind].p2.x ) > 2.) && (fabs(path.path[t1_ind].p2.y - _path[t2_ind].p2.y) > 2.) )
        return 0;
    else
        if(verbose > 2)
            cout << "same_t 4.0" << endl;

    logs << INFO << "Same path";
    return 1 ;
}

/*
 * Return 1 if the same path else 0.
 */
bool Path::checkSamePath2(deque<sTrajEl_t>& path){
    int i = path.size(), j = _path.size();

    if(i != j)
        return false;

    while (i >= 0 && j >= 0) {
        if(checkSameObs(path[i].obs, _path[j].obs)){
            i--;
            j--;
        }else
            return false;
    }
    return true;
}

/*
 * Return 1 if the robot is block else 0.
 */
int Path::checkRobotBlock() {
    static sPt_t pos[10] = { { 0., 0. } };
    static int pt = 0;
    static int block = 0;
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
            if(block == 0){
                logs << DEBUG << "Robot is blocked";
                block = 1;
            }
            return 1;
        }
        lastTime = millis();
        block = 0;
    }

    return 0;
}

void Path::updateNoHaftTurn() {
    int i;
    sNum_t r;
    sPt_t pt = statuses.getLastPosXY(ELT_PRIMARY);
    float theta = statuses.getLastOrient(ELT_PRIMARY);
    static sObs_t _obs[3];

    r = statuses.getLastSpeed(ELT_PRIMARY); //15=speed
    r /= 3;
    if (r > 15)
        r = 15;

    for (i = 0; i < 3; i++) {
        obs[N - i - 5].c.x = pt.x + (r) * cos(theta + i * M_PI_2);
        obs[N - i - 5].c.y = pt.y + (r) * sin(theta + i * M_PI_2);
        obs[N - i - 5].active = 1;
        obs[N - i - 5].r = r - 0.5;
        if(_obs[i].c.x != obs[N - i - 5].c.x && _obs[i].c.y != obs[N - i - 5].c.y){
            obs_updated[N - i - 5]++;
            _obs[i] = obs[N - i - 5];
        }
    }
    if (r < 0.1) {
        for (i = 1; i < 4; i++)
            obs[N - i - 4].active = 0;
    }
}
