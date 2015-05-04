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

//#define DEBUG_OBJ

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
float Obj::update(const bool axle,  std::vector<astar::sObs_t>& obs, const int robot) {
    sPath_t path_loc;
    Point2D<float> posRobot(obs[robot].c.x, obs[robot].c.y);
    int N = obs.size();
    std::vector<int> list;
    int result;

    _dist = -1;
    Point2D<float> p = projectPointInObs(posRobot, obs);
    obs[robot].c = {p.x, p.y};

    for (sObjEntry_t i : _access) {
#ifdef DEBUG_OBJ
        logs << DEBUG << "Access type : " << objAccess(i.type);
#endif
        switch(i.type){
            case E_POINT :
                obs[obs.size() - 1].c = {i.pt.p.x, i.pt.p.y};
                if(axle)
                    updateEndTraj(i.pt.angle, &i.pt.p, i.radius, obs);
                    p = {obs[robot].c.x,obs[robot].c.y};
                    p = projectPointInObs(p, obs);
                    obs[robot].c = {p.x, p.y};
                break;
            case E_CIRCLE:
                obs[obs.size() - 1].c = {i.cir.c.x, i.cir.c.y};
                if(axle){
                    obs[N - 2].active = 0;
                    obs[N - 3].active = 0;
                    obs[N - 4].active = 0;
                }
                break;
            case E_SEGMENT:
                //TODO
                break;
            default:
                logs << ERR << "Unknown type of access to objective";
        }

#ifdef DEBUG_OBJ
        logs << DEBUG << "Current position is : (" << obs[0].c.x << " ; " << obs[0].c.y << ")";
        logs << DEBUG << "Current access point chose is : (" << obs[N - 1].c.x << " ; " << obs[N - 1].c.y << ")";
#endif


        for(unsigned int j : _num_obs){ // Disable necessary obstacle to achieve the objective
            obs[j].active = 0;
#ifdef DEBUG_OBJ
            logs << DEBUG << "Disable obstacle number : " << j;
#endif
        }

        if(i.type == E_CIRCLE){
            Point2D<float> dest(obs[obs.size() - 1].c.x, obs[obs.size() - 1].c.y);

            while((result = checkPointInObs(dest, obs)) != 0){
                logs << DEBUG << "Disable obstacle number : " << result;
                list.push_back(result);
                obs[result].active = 0;
            }
        }

        astar::fill_tgts_lnk(obs);
        a_star(A(robot), A(N-1), &path_loc);

        if(i.type == E_CIRCLE){
            for(int i : list){
                obs[i].active = 1;
            }
        }

        for(unsigned int j : _num_obs){ // Reactivation obstacle
            obs[j].active = 1;
#ifdef DEBUG_OBJ
            logs << DEBUG << "Reactivation of obstacle number :" << j;
#endif
        }

        if (path_loc.dist == 0){
#ifdef DEBUG_OBJ
            logs << INFO << "A* return null distance";
#endif
        }
        else if (_dist > path_loc.dist || _dist == -1) { // The new path is better to access the objective
            _path = path_loc;

            if(i.type == E_CIRCLE){
                if(_path.path[_path.path_len - 1].p1.distanceTo(_path.path[_path.path_len - 1].p2) < i.cir.r){
                    continue;
                }
                else{
                    Point2D<float> pt = _path.path[_path.path_len - 1].p1;
                    Circle2D<float> cir(i.cir.c.x, i.cir.c.y, i.cir.r);

                    pt = cir.projecteSup(pt, 0.1);
                    if(checkPointInObs(pt, obs)){
                        continue;
                    }
                    _path.path[_path.path_len - 1].p2 = pt;
                    _dist = path_loc.dist - i.cir.r;
                    Vector2D<float> v1(1,0), v2(cir.c, pt);
                    _access_select_angle = v1.angle(v2) + M_PI; //M_PI because reference inverse
                }
            }
            else if(i.type == E_POINT){
                _access_select_angle = i.pt.angle + M_PI; //M_PI because reference inverse
                _dist = path_loc.dist;
            }

            _access_select = _path.path[_path.path_len - 1].p2;
        }
    }

    obs[robot].c = {posRobot.x, posRobot.y}; // Restore the original position

    return _dist;
}

int Obj::updateDestPointOrient(const vector<Actuator>& act){
    unsigned int i;

    if(act.empty())
        return -1;

    for(i = 0 ; i < act.size() ; i++){
        if( act[i].type == _type)
            break;
    }

    if(i == act.size()){
        logs << ERR << "Any actuator define for this objective";
        return -1;
    }

    if(act[i].active.size() != act[i].angle.size()){
        logs << ERR << "Different size : active=" << act[i].active.size() << "and angle=" << act[i].angle.size();
    }

    if(act[i].active.empty())
        logs << INFO << "Any actuator available";

    if(act[i].active[0]) //TODO For the moment choose the first find
        _access_select_angle += act[i].angle[0];


    return 0;
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
        case E_CUP:
        case E_SPOT:
            ratio = 1/_dist * 100; //TODO
            break;
        default:
            return (-1);
    }
    return ratio * (1. - _done);
}


void Obj::print(){
    logs << DEBUG << fixed << setprecision(2) << "type : " << objType() << " : " << objState();
    if(_state == ACTIVE){
        if(_dist > 0)
            logs << " : " << _dist << "cm to go" << _access_select << "(" << _access_select_angle*180/M_PI << "°)";
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
