/*
 * path.cpp
 *
 *  Created on: 18 févr. 2015
 *      Author: Sebastien Malissard
 */


#include "path.h"

#include <cmath>
#include <iostream>
#include <cstring>
#include "GeometryTools.h"

#include "bn_intp.h"
#include "messages-statuses.h"
#include "messages-locomotion.h"

#include "a_star_tools.h"
#include "a_star.h"
#include "ai_tools.h"
#include "obj_tools.h"
#include "time_tools.h"
#include "tools.h"
#include "statuses.h"
#include "communications.h"

extern "C"{
#include "millis.h"
#include "roles.h"
#include <unistd.h>
}


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

/*
    static unsigned int prevTime2 = 0;
    if(millis() - prevTime2 > 100){
        updateNoHaftTurn();
        prevTime2 = millis();
    }
*/
}

/*
 * Send the path to the robot.
 * Every parameter that can be calculated automatically, the calculation is performed.
 * Try to send MAX_RETRIES if failed.
 */
void Path::sendRobot(bool holo, float thetaEnd) {
    static sPath_t path;
    static deque<sTrajOrientEl_t> pathOrient;

    logs << INFO << "Starting sendRobot()";

    if(!_path.empty() && !_path_orient.empty()){
        logs << ERR << "Try to send a path : _path and _path_orient are not empty";
        return;
    }
    if(_path.empty() && _path_orient.empty()){
        logs << ERR << "Try to send a path : _path and _path_orient are empty ";
        return;
    }

    length();

    if(holo)
        convPathToPathOrient(thetaEnd);

    if (!_path.empty() && (!checkSamePath(path) || checkRobotBlock())){
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
    else if(!_path_orient.empty() && (!checkSamePathOrient(pathOrient) || checkRobotBlock())){
            pathOrient.clear();

            for(const sTrajOrientEl_t& i : _path_orient)
                pathOrient.push_back(i);

            net.sendPathOrient(_path_orient);

            for(unsigned int i = 0 ; i < _path_orient.size() ; i++)
                printElTrajOrient(i);
    }
}

/*
 * Stop the primary robot
 */
void Path::stopRobot(bool holo) {
    sMsg msg;

    memset(&msg, 0, sizeof(msg));

    logs << WAR << "Stop robot";

    if (holo) {
        msg.header.destAddr = role_get_addr(ROLE_PRIM_PROPULSION);
        msg.header.type = E_PROP_STOP;
        msg.header.size = 0;

        bnSendBlock(msg, "stop robot");
    }
    else { // for backward compatibility
        Point2D<float> posRobot = statuses.getLastPosXY(ELT_PRIMARY);
        float theta = statuses.getLastOrient(ELT_PRIMARY);
        sTrajEl_t traj = sTrajEl_t{posRobot, posRobot, {{posRobot.x, posRobot.y}, 0, 0, 1, 0}, 0, 0, 0 };

        clear();
        _path.push_back(traj);

        sendRobot(holo, theta);
    }
}


void Path::go2PointOrient(const Point2D<float> &dest, vector<astar::sObs_t>& /*obs*/, float angle){
    Point2D<float> posRobot = statuses.getLastPosXY(ELT_PRIMARY);//posRobot(obs[0].c.x, obs[0].c.y);

    clear();
    logs << INFO << "gottoPoint" << posRobot.x << " " << posRobot.y;
    sTrajEl_t traj = sTrajEl_t{{posRobot.x, posRobot.y}, dest, {{dest.x, dest.y}, 0., 0, 1, 0}, 0, 0, 0 };

    _path.push_back(traj);
    sendRobot(true, angle);
}

/*
 * The robot go to the destination point.
 * "f" to force the robot to go, even if the destination point is in obstacle or if there are obstacles in the trajectory.
 */
void Path::go2Point(const Point2D<float> &dest, const bool f, vector<astar::sObs_t>& obs, bool holo){
    sPath_t path;
    Point2D<float> posRobot(obs[0].c.x, obs[0].c.y);
    int N = obs.size();

    obs[N-1].c = {dest.x, dest.y};
    logs << DEBUG << "go2Point() : position : " << obs[0].c.x << ", " << obs[0].c.y << " ; destination : " << obs[N-1].c.x << ", " << obs[N-1].c.y;

    Point2D<float> p1(obs[0].c.x, obs[0].c.y),  p2(obs[N-1].c.x, obs[N-1].c.y);
    if(p1.distanceSqTo(p2) < 2.*2.)
        return;

    if(f){
        clear();
        sTrajEl_t traj = sTrajEl_t{{obs[0].c.x, obs[0].c.y}, dest, {{dest.x, dest.y}, 0., 0, 1, 0}, 0, 0, 0 };

        _path.push_back(traj);
        sendRobot(holo, atan2(dest.y - obs[0].c.y, dest.x - obs[0].c.x ));
    }
    else {
        Point2D<float> p = projectPointInObs(posRobot, obs);
        obs[0].c = {p.x, p.y};

        astar::fill_tgts_lnk(obs);
        a_star(A(0), A(N-1), &path);
        if (path.path) {
            logs << INFO << "go2Point() : New path from 0a to " <<  N-1 << " (" << path.dist << ", " << path.path_len << " steps )";
            addPath2(path);
            sendRobot(holo, atan2(_path[_path.size()-1].p2.y - _path[_path.size()-1].p1.y, _path[_path.size()-1].p2.x - _path[_path.size()-1].p1.x));
            free(path.path);
        }
        else {
            logs << INFO << "go2Point() : No path from 0a to " << N - 1;
        }
        obs[0].c = {posRobot.x, posRobot.y};
    }
}


void Path::followPath(vector <astar::sObs_t>& , vector <astar::iABObs_t> &, bool ) { // todo tableau statique //for traj prog
    clear();
/*
   //copier _obs dans obs
    //TODO if there are an adversaire

    for (unsigned int i = 0; i < l.size()-1; i++) {
        sTrajEl_t el;

        sSeg_t *s = tgt(l[i], l[i + 1]);

        el.p1 = {s->p1.x, s->p1.y};
        el.p2 = {s->p2.x, s->p2.y};
        el.obs.active = 1;
        el.obs.c = obs[O(l[i + 1])].c;
        el.obs.moved = 1;
        el.obs.r = fabs(obs[O(l[i + 1])].r) * (1 - 2 * DIR(l[i + 1]));
        el.seg_len = 0; // FIXME
        el.arc_len = 0; // FIXME
        el.sid = i;

        _path.push_back(el);
    }

    sendRobot(holo);
    */
}

/*
 * Converts a path in path orient.
 * This path orient can't directly used by a holonomic robot (theta set to 0 and time too)
 */
void Path::convPathToPathOrient(float thetaEnd){
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

    computePathHolonomic(thetaEnd);
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
void Path::computePathHolonomic(float theta_end_obj){
    float theta = statuses.getLastOrient(ELT_PRIMARY);
    float deltaTheta = getPrincipalAngleValue(theta_end_obj - theta);
    float distTotal = 0, dist = 0;
    uint32_t time_us = bn_intp_micros2s(micros());

    for(unsigned int i = 0 ; i < _path_orient.size() ; i++){
        distTotal += _path_orient[i].seg_len;
        distTotal += _path_orient[i].arc_len;
    }

    float timeLinear = distTotal/MAX_SPEED;
    float timeAngular = fabs(deltaTheta/MAX_SPEED_ROT);

    float Time = timeLinear>timeAngular?timeLinear:timeAngular;

    if(distTotal > 0.01){
        for(unsigned int i = 0 ; i < _path_orient.size() ; i++){
            _path_orient[i].t1 = (uint32_t)((dist/distTotal)*Time*1e6) + time_us; //in us
            _path_orient[i].theta1 = getPrincipalAngleValue((dist/distTotal)*deltaTheta + theta);
            dist += _path_orient[i].seg_len;

            _path_orient[i].t2 = (uint32_t)((dist/distTotal)*Time*1e6) + time_us; //in us
            _path_orient[i].theta2 = getPrincipalAngleValue((dist/distTotal)*deltaTheta + theta);
            dist += _path_orient[i].arc_len;

            _path_orient[i].rot1_dir = deltaTheta > 0;
            _path_orient[i].rot2_dir = deltaTheta > 0;
        }
    }
    else if(_path_orient.size() == 1){ //rotation
        logs << ERR << "Rotation";
        _path_orient[0].t1 = (uint32_t) time_us; //in us
        _path_orient[0].t2 = (uint32_t) (Time*1e6) + time_us; //in us

        _path_orient[0].theta1 = getPrincipalAngleValue(theta);
        _path_orient[0].theta2 = getPrincipalAngleValue(theta + deltaTheta);

        _path_orient[0].rot1_dir = deltaTheta > 0;
        _path_orient[0].rot2_dir = deltaTheta > 0;
    }else{
        logs << ERR << "Strange trajectory : angle doesn't computed";
    }

}

PointOrient2D<float> Path::getPosOrient(uint32_t _time_us /*synchronize time*/) { // Be careful, return an ideal the position orient
    PointOrient2D<float> po;

    if (!_lastTrajPosSpd.empty()) {
        if ((long) _time_us - (long) _lastTrajPosSpd.front().t < 0) {
            return { {nanf(""), nanf("")}, nanf("")};
        }

        for (auto i = _lastTrajPosSpd.begin(); i != _lastTrajPosSpd.end() - 1; ++i) {
            if (i->t >= _time_us && (i+1)->t < _time_us) {
                float d = 1/2*((i+1)->v - i->v)/((i+1)->t - i->t)*(_time_us - i->t)*(_time_us - i->t) + i->v*(_time_us - i->t);
                float theta = 1/2*((i+1)->theta - i->theta)/((i+1)->t - i->t)*(_time_us - i->t)*(_time_us - i->t) + i->theta*(_time_us - i->t);

                switch (i->type_e) {
                case TRAJSTEP_LINE: {
                    Vector2D<float> v1(1, 0), v2(i->p, (i+1)->p);
                    float o = v1.angle(v2);
                    return { {i->p.x + d*cosf(o), i->p.y + d*sinf(o)}, theta};
                }

                case TRAJSTEP_ARC: {
                    Vector2D<float> v1(1, 0), v2({i->type.arc.c.c.x, i->type.arc.c.c.y}, (i+1)->p);
                    float o = v1.angle(v2) + d/i->type.arc.c.r;
                    return { {i->type.arc.c.c.x + abs(i->type.arc.c.r)*cosf(o), i->type.arc.c.c.y + abs(i->type.arc.c.r)*sinf(o)}, theta};
                }

                default:
                    logs << ERR << "Unknown type of trajectory element";
                }
            }
        }


        return {_lastTrajPosSpd.back().p, _lastTrajPosSpd.back().theta};
    }

    return { {nanf(""), nanf("")}, nanf("")};

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
                               << " ; theta1=" << _path_orient[num].theta1*180/M_PI<< "° theta2=" << _path_orient[num].theta2*180/M_PI << "°"
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
            _path[i].seg_len = _path[i].p1.distanceTo(_path[i].p2);
        }

        _path[_path.size() - 1].arc_len = 0;
        _path[_path.size() - 1].seg_len = _path[_path.size() - 1].p1.distanceTo(_path[_path.size() - 1].p2);
    }
    else if(!_path_orient.empty()){
        for (unsigned int i = 0; i < _path_orient.size() - 1; i++) {
            Circle2D<float> c(_path_orient[i].obs.c.x, _path_orient[i].obs.c.y, _path_orient[i].obs.r);
            Point2D<float> p1(_path_orient[i].p2.x, _path_orient[i].p2.y), p2(_path_orient[i+1].p1.x, _path_orient[i+1].p1.y);
            _path_orient[i].arc_len = c.arcLenght(p1, p2);
            _path_orient[i].seg_len = _path_orient[i].p1.distanceTo(_path_orient[i].p2);
        }

        _path_orient[_path_orient.size() - 1].arc_len = 0;
        _path_orient[_path_orient.size() - 1].seg_len = _path_orient[_path_orient.size() - 1].p1.distanceTo(_path_orient[_path_orient.size() - 1].p2);
    }
}

/*
 * Return true(1) if the 2 obs are identical else false(0)
 */
bool Path::checkSameObs(astar::sObs_t& obs1, astar::sObs_t& obs2){
    return abs(obs1.r - obs2.r) < 1.f && abs(obs1.c.x - obs2.c.x) < 1.f && abs(obs1.c.y - obs2.c.y) < 1.f;
    }

/*
 * Return 1 if the same path else 0.
 */
bool Path::checkSamePath(sPath_t &path){
    unsigned int t1_ind = path.path_len;
    unsigned int t2_ind = _path_len;

    if(path.path_len==0 || _path.size()==0)
        return false;

    while ((int)t1_ind > 0 &&  (int)t2_ind > 0) { //pb si un step est terminé
        if (checkSameObs((path.path[t1_ind-1].obs), (_path[t2_ind-1].obs)) ){
          t1_ind--;
           t2_ind--;
           }
       else return false;
    }

    return true;
}

/*
 * Return 'true' if the same path else 'false'.
 */
bool Path::checkSamePathOrient(deque<sTrajOrientEl_t>& path){
    int i = path.size() - 1, j = _path_orient.size() - 1;

    while (i >= 0 && j >= 0) {
        if(checkSameObs(path[i].obs, _path_orient[j].obs)){
            i--;
            j--;
        }else
            return false;
    }

    if (abs(path[i+1].theta2 - _path_orient[j+1].theta2) < M_PI/180.f && (path[i+1].p2 - _path_orient[j+1].p2).normSq() < 1.f)
        return true;
    else
        return false;
}

/*
 * Return 1 if the robot is block else 0.
 */
int Path::checkRobotBlock() {
    static Point2D<float> pos[10] = { { 0., 0. } };
    static int pt = 0;
    static int block = 0;
    static unsigned int lastTime = 0;
    int i, cpt = 0;
    Point2D<float> pos_robot = statuses.getLastPosXY(ELT_PRIMARY);

    if (fabs(time_diff(millis(), lastTime)) > 200) {
        pos[pt] = pos_robot;
        pt++;
        pt = pt % 10;
        for (i = 0; i < 10; i++) {
            if (pos_robot.distanceTo(pos[i]) < 1.)
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

void Path::updateNoHaftTurn(vector<astar::sObs_t>& obs, std::vector<uint8_t> obs_updated) {
    int i, N = obs.size();
    sNum_t r;
    Point2D<float> pt = statuses.getLastPosXY(ELT_PRIMARY);
    float theta = statuses.getLastOrient(ELT_PRIMARY);
    static astar::sObs_t _obs[3];

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
