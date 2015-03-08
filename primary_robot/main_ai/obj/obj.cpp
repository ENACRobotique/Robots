/*
 * obj.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#include "obj.h"

#include <iostream>

#include <geometry_tools.h>
#include "math_ops.h"
#include "obj_tools.h"
#include "a_star.h"
#include "tools.h"
#include "ai_tools.h"

extern "C"{
#include "millis.h"
}

int testInObs(sPt_t *p) { //retourne le numéro de l'obstable si la position est a l'interieur de celui ci
    //FIXME si le robot dans plusieurs obstable
    int i;
    sNum_t dist;
    for (i = 1; i < N - 1; i++) {
        if (obs[i].active == 0)
            continue;
        distPt2Pt(&obs[i].c, p, &dist);
        if (dist < obs[i].r) {
            //printf("Le robot est dans l'obstacle n=%i, robs=%f, xobs=%f, yobs=%f, currentpos x=%f, y=%f\n",i,obs[i].r,obs[i].c.x,obs[i].c.y, _current_pos.x, _current_pos.y);
            return i;
        }
    }
    return 0;
}

Obj::Obj() : _type(E_NULL), _point(-1), _state(ACTIVE), _dist(-1), _time(-1), _done(0){
}

Obj::Obj(eObj_t type) : _type(type), _point(-1), _state(ACTIVE), _dist(-1), _time(-1), _done(0){
}

Obj::Obj(eObj_t type, vector<unsigned int> &numObs, vector<sObjEntry_t> &entryPoint) :
        _type(type), _point(-1), _state(ACTIVE), _dist(-1), _time(-1), _done(0){

    for(unsigned int i : numObs){
        _num_obs.push_back(i);
    }

    for(sObjEntry_t i : entryPoint){
        _access.push_back(i);
    }
}

Obj::~Obj() {
    // TODO Auto-generated destructor stub
}



void Obj::addAccess(sObjEntry_t &access){
    _access.push_back(access);
}

/*
 * Computing the shortest distance between the position of the robot and the objective entry point.
 * And update the others parameters
 * //TODO Considering the orientation of the robot
 */
sNum_t Obj::update(sPt_t posRobot) {
    int g, m, n;
    sPath_t path_loc;

    _dist = -1;
    obs[0].c = statuses.getLastPosXY(ELT_PRIMARY);
logs << INFO << "--------------------------------------------------------------";
    if ((n = testInObs(&obs[0].c)) != 0) {
        projectPoint(posRobot.x, posRobot.y, obs[n].r, obs[n].c.x, obs[n].c.y, &obs[0].c);
        logs << INFO << "Robot in obstacle : " << n;
    }

    for (sObjEntry_t i : _access) {
        obs[0].c = statuses.getLastPosXY(ELT_PRIMARY);
        switch(i.type){
            case E_POINT :
                logs << DEBUG << "Access type POINT";
                obs[N - 1].c = i.pt.p;
#ifdef NON_HOLONOMIC
                updateEndTraj(i.pt.angle, &i.pt.p, i.radius);
#endif
            break;
            case E_CIRCLE:
                logs << DEBUG << "Access type CIRCLE";
                obs[N - 1].c = i.cir.c;
#ifdef NON_HOLONOMIC
                obs[N - 2].active = 0;
                obs[N - 3].active = 0;
                obs[N - 4].active = 0;
                obs[N - 5].active = 0;
                obs[N - 6].active = 0;
                obs[N - 7].active = 0;
#endif
                break;
            case E_SEGMENT:
                //TODO
                break;
            default:
                logs << ERR << "Unknown type of access to objective";
        }

#ifdef NON_HOLONOMIC
        if ((g = testInObs(&obs[0].c)) != 0) { //Projection if the robot is inside a "circle of end trajectory"
            projectPoint(posRobot.x, posRobot.y, obs[g].r, obs[g].c.x, obs[g].c.y, &obs[0].c);
            if ((m = testInObs(&obs[0].c)) != 0) { //Cas la projection se retrouve dans un obstacle après la premier projection
                logs << ERR << "Fix this projection inside an obstacle n°" << m; //FIXME or no : investigate
                continue;
            }
        }
#endif
        logs << DEBUG << "Current position is : " << obs[0].c.x << " : " << obs[0].c.y;
        logs << DEBUG << "Current access point chose is : " << obs[N - 1].c.x << " : " << obs[N - 1].c.y;
        if(!_num_obs.empty())
            logs << DEBUG << "Current obstacle position is : " << obs[_num_obs.front()].c.x << " : " << obs[_num_obs.front()].c.y;

        //deactivate obs
        for(unsigned int j : _num_obs){
            obs[j].active = 0;
            logs << DEBUG << "Deactivation of obstacle number :" << j;
        }

        fill_tgts_lnk();
        a_star(A(0), A(N-1), &path_loc);

        //reactivate obs
        for(unsigned int j : _num_obs){
            obs[j].active = 1;
            logs << DEBUG << "Reactivation of obstacle number :" << j;
        }

        if (path_loc.dist == 0)
            logs << INFO << "A* return null distance";
        else if (_dist > path_loc.dist || _dist == -1) {
            _dist = path_loc.dist;
            _path = path_loc;
            _access_select = obs[N - 1].c;

            if( i.type == E_CIRCLE){
                float dist = 0;
                distPt2Pt(&_path.path[_path.path_len - 1].p1, &_path.path[_path.path_len - 1].p2, &dist);
                if(dist < i.cir.r)
                    logs << ERR << "Very strange path"; //FIXME
                else{
                    sPt_t pt = _path.path[_path.path_len - 1].p1;
                    if(projPtOnCircle(&i.cir.c, i.cir.r, &pt) < 0)
                        logs << ERR << "Projection of point";
                    _path.path[_path.path_len - 1].p2 = pt;
                }
                _access_select = _path.path[_path.path_len - 1].p2;
                //_dist -= (i.cir.r + R_ROBOT);
            }
        }//else not better, conserve the previous
    }

    return _dist;
}

float Obj::getDist() const{
    return _dist;
}

sPath_t Obj::getPath() const{
    return _path;
}

sPt_t Obj::getDestPoint() const{
    return _access_select;
}

eStateObj_t Obj::getState() const{
    return _state;
}

sNum_t Obj::getYield(void){
    sNum_t ratio = 0.;

    if (_dist == 0) { //FIXME on est sur le point donc valeur max : Erreur ou objectif atteint
        logs << ERR << "getYield : distance null";
        return (-1);
    }

    if (_time > (END_MATCH - (millis() - _start_time))) { //insufficient time remaining
        _state = NO_TIME;
        return 0;
    }

    switch (_type) {
        case E_CLAP:
        case E_SPOT:
            ratio = 1/_dist * 100; //TODO
            break;
        default:
            return (-1);
    }
    return ratio * (1. - _done);
}

