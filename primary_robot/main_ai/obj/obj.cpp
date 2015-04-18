/*
 * obj.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#include "obj.h"

#include <iostream>

#include "obj_tools.h"
#include "a_star.h"
#include "tools.h"
#include "ai_tools.h"
#include "GeometryTools.h"
#include <iomanip>

extern "C"{
#include "millis.h"
}

#ifndef HOLONOMIC
#error "HOLONOMIC must be defined"
#endif

int testInObs() { //retourne le numéro de l'obstable si la position est a l'interieur de celui ci
    //FIXME si le robot dans plusieurs obstable
    Point2D<float>p = statuses.getLastPosXY(ELT_PRIMARY);
    int i;
    float dist;
    for (i = 1; i < N - 1; i++) {
        if (obs[i].active == 0)
            continue;
        dist = p.distanceTo({obs[i].c.x, obs[i].c.y});
        if (dist < obs[i].r) {
            //printf("Le robot est dans l'obstacle n=%i, robs=%f, xobs=%f, yobs=%f, currentpos x=%f, y=%f\n",i,obs[i].r,obs[i].c.x,obs[i].c.y, _current_pos.x, _current_pos.y);
            return i;
        }
    }
    return 0;
}

Obj::Obj() : _type(E_NULL), _point(-1), _state(ACTIVE), _access_select_angle(0), _dist(-1), _time(-1), _done(0){
}

Obj::Obj(eObj_t type) : _type(type), _point(-1), _state(ACTIVE), _access_select_angle(0), _dist(-1), _time(-1), _done(0){
}

Obj::Obj(eObj_t type, vector<unsigned int> &numObs, vector<sObjEntry_t> &entryPoint) :
        _type(type), _point(-1), _state(ACTIVE), _access_select_angle(0), _dist(-1), _time(-1), _done(0){

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
float Obj::update(sPt_t posRobot) {
    int n;
#if !HOLONOMIC
    int g, m;
#endif
    sPath_t path_loc;
    Point2D<float> posRobotp = statuses.getLastPosXY(ELT_PRIMARY);

    _dist = -1;
    obs[0].c = {posRobotp.x, posRobotp.y};

    if ((n = testInObs()) != 0) {
        Point2D<float> p(posRobot.x, posRobot.y);
        Circle2D<float> c(obs[n].c.x, obs[n].c.y, obs[n].r);
        p = c.projecte(p);
        obs[0].c = {p.x, p.y};
#ifdef DEBUG_OBJ
        logs << INFO << "Robot in obstacle : " << n;
#endif
    }

    for (sObjEntry_t i : _access) {
        obs[0].c = {posRobotp.x, posRobotp.y};
        switch(i.type){
            case E_POINT :
#ifdef DEBUG_OBJ
                logs << DEBUG << "Access type POINT";
#endif
                obs[N - 1].c = {i.pt.p.x, i.pt.p.y};
#if !HOLONOMIC
                updateEndTraj(i.pt.angle, &i.pt.p, i.radius);
#endif
            break;
            case E_CIRCLE:
#ifdef DEBUG_OBJ
                logs << DEBUG << "Access type CIRCLE";
#endif
                obs[N - 1].c = {i.cir.c.x, i.cir.c.y};
#if !HOLONOMIC
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

#if !HOLONOMIC
        if ((g = testInObs(&obs[0].c)) != 0) { //Projection if the robot is inside a "circle of end trajectory"
            projectPoint(posRobot.x, posRobot.y, obs[g].r, obs[g].c.x, obs[g].c.y, &obs[0].c);
            if ((m = testInObs(&obs[0].c)) != 0) { //Cas la projection se retrouve dans un obstacle après la premier projection
                logs << ERR << "Fix this projection inside an obstacle n°" << m; //FIXME or no : investigate
                continue;
            }
        }
#endif
#ifdef DEBUG_OBJ
        logs << DEBUG << "Current position is : " << obs[0].c.x << " : " << obs[0].c.y;
        logs << DEBUG << "Current access point chose is : " << obs[N - 1].c.x << " : " << obs[N - 1].c.y;
        if(!_num_obs.empty())
            logs << DEBUG << "Current obstacle position is : " << obs[_num_obs.front()].c.x << " : " << obs[_num_obs.front()].c.y;
#endif

        //deactivate obs
        for(unsigned int j : _num_obs){
            obs[j].active = 0;
#ifdef DEBUG_OBJ
            logs << DEBUG << "Deactivation of obstacle number :" << j;
#endif
        }

        fill_tgts_lnk();
        a_star(A(0), A(N-1), &path_loc);

        //reactivate obs
        for(unsigned int j : _num_obs){
            obs[j].active = 1;
#ifdef DEBUG_OBJ
            logs << DEBUG << "Reactivation of obstacle number :" << j;
#endif
        }

        if (path_loc.dist == 0){
#if DEBUG_OBJ
            logs << INFO << "A* return null distance";
#endif
        }
        else if (_dist > path_loc.dist || _dist == -1) {
            _dist = path_loc.dist;
            _path = path_loc;
            _access_select = {obs[N - 1].c.x, obs[N - 1].c.y};

            if( i.type == E_CIRCLE){
                if(_path.path[_path.path_len - 1].p1.distanceSqTo(_path.path[_path.path_len - 1].p2) < i.cir.r){
#if DEBUG_OBJ
                    logs << ERR << "Very strange path"; //FIXME deux obstable sont très provhe
#endif
                }
                else{
                    Point2D<float> pt = _path.path[_path.path_len - 1].p1;
                    Circle2D<float> cir(i.cir.c.x, i.cir.c.y, i.cir.r);
                    if(cir.c.distanceTo(pt) == 0)
                        logs << ERR << "Can't project point";
                    pt = cir.projecte(pt);
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

Point2D<float> Obj::getDestPoint() const{
    return _access_select;
}

float Obj::getDestPointOrient() const{
    return _access_select_angle;
}

eStateObj_t Obj::getState() const{
    return _state;
}

float Obj::getYield(const unsigned int start_time){
    float ratio = 0.;

    if (_dist == 0) { //FIXME on est sur le point donc valeur max : Erreur ou objectif atteint
        logs << ERR << "getYield : distance null";
        return (-1);
    }

    if (_time > (END_MATCH - (millis() - start_time))) { //insufficient time remaining
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


void Obj::print(){
    logs << DEBUG << fixed << setprecision(2) << "type : " << obsType() << " : " << obsState();
    if(_state == ACTIVE){
        if(_dist > 0)
            logs << " : " << _dist << "cm to go" << _access_select << "(" << _access_select_angle << "°)";
        else
            logs << " : no path found";
    }
}

/*
void updateRatioObj(int numObj, int robot) { //robot = 1 to 3
    static int mat[3][NB_OBJ] = { { 0, 0 } };  //3 is the number of other robot

    if (!mat[robot - 1][numObj]) {
        //    listObj[numObj].done += (1 - listObj[numObj].done)/2;
        mat[robot - 1][numObj] = 1;
    }
}
*/

/*
 void checkRobot2Obj(void){
 int i, j, k;
 float dist;
 static int tab[NB_OBJ][2] ={{0, 0}};

 for(i = 0 ; i < NB_OBJ ; i++){
 if(tab[i][0]){
 tab[i][0] = 0;
 listObj[i].active = tab[i][1];
 }
 for(j = 1 ; j < 4 ; j++){
 for(k = 0 ; k < listObj[i].nbObs ; k++){
 distPt2Pt(&obs[listObj[i].numObs[k]].c, &obs[j].c, &dist);
 if( (obs[listObj[i].numObs[k]].r + obs[j].r - R_ROBOT) > (dist - ERR_DIST) ){
 updateRatioObj(i, j);
 tab[i][0] = 1;
 tab[i][1] = listObj[i].active;
 listObj[i].active = 0;
 }
 }
 }
 }
 }
 */
