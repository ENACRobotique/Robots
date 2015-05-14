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

#define DEBUG_OBJ

Obj::Obj() : _type(E_NULL), _typeAct(ActuatorType::ANY), _get(true), _point(-1), _state(ACTIVE), _access_select_angle(0), _actuator_select(-1),_dist(-1), _time(-1), _done(0){
}

Obj::Obj(eObj_t type, ActuatorType typeAct, bool get) : _type(type),  _typeAct(typeAct), _get(get), _point(-1), _state(ACTIVE), _access_select_angle(0), _actuator_select(-1), _dist(-1), _time(-1), _done(0){
}

Obj::Obj(eObj_t type, ActuatorType typeAct, bool get, vector<unsigned int> &numObs, vector<sObjEntry_t> &entryPoint) :
        _type(type), _typeAct(typeAct), _get(get), _point(-1), _state(ACTIVE), _access_select_angle(0), _actuator_select(-1), _dist(-1), _time(-1), _done(0){

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
 */
float Obj::update(const bool /*axle*/,  std::vector<astar::sObs_t>& obs, const int robot) {
    sPath_t path_loc;
    Point2D<float> posRobot(obs[robot].c.x, obs[robot].c.y);
    int N = obs.size();
    std::vector<int> list;
    int result;

    _dist = -1;

    Point2D<float> p = projectPointInObs(posRobot, obs);

    if(checkPointInObs(p, obs)){ //Robot is block
        for(unsigned int i = 4 ; i < obs.size() - 1 ; i++){
            if(obs[i].active && obs[i].moved){
#ifdef DEBUG_OBJ
                logs << DEBUG << "Disable obstacle number 2 : " << i;
#endif
                list.push_back(i);
                obs[i].active = 0;
            }
        }
    }

    obs[robot].c = {p.x, p.y};

    for (unsigned int i = 0 ; i < _access.size(); i++) {
#ifdef DEBUG_OBJ
        logs << DEBUG << "Access type : " << objAccess(_access[i].type);
#endif
        switch(_access[i].type){
            case E_POINT :
                obs[obs.size() - 1].c = {_access[i].pt.p.x, _access[i].pt.p.y};
                updateEndTraj(_access[i].pt.angle, &_access[i].pt.p, _access[i].radius, obs);
                p = {obs[robot].c.x,obs[robot].c.y};
                p = projectPointInObs(p, obs); //TODO pas top !!
                obs[robot].c = {p.x, p.y};
                break;
            case E_CIRCLE:
                {
                    Point2D<float> d;
                    d = projectPointInLimitPlayground({_access[i].cir.c.x, _access[i].cir.c.y},  R_ROBOT);
                    logs << INFO << "demande de projection point destination limit playground resultat x="
                            << d.x << " y=" << d.y;
                    obs[obs.size() - 1].c = {d.x, d.y};
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

        if(_access[i].type == E_CIRCLE){
            Point2D<float> dest(obs[obs.size() - 1].c.x, obs[obs.size() - 1].c.y);

            while((result = checkPointInObs(dest, obs)) != 0){
                if(obs[result].moved == 0){
                    dest = projectPointInObs(dest, obs);
                    obs[obs.size() - 1].c.x = dest.x;
                    obs[obs.size() - 1].c.y = dest.y;
                    break;
                }
#ifdef DEBUG_OBJ
                logs << DEBUG << "Disable obstacle number : " << result;
#endif
                list.push_back(result);
                obs[result].active = 0;
            }
        }

        astar::fill_tgts_lnk(obs);
        a_star(A(robot), A(N-1), &path_loc);

        for(int i : list){
            obs[i].active = 1;
#ifdef DEBUG_OBJ
            logs << DEBUG << "Reactivation of obstacle number :" << i;
#endif
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
            if(_path.path){
                free(_path.path);
            }

            _path = path_loc;

            if(_access[i].type == E_CIRCLE){
                if(_path.path[_path.path_len - 1].p1.distanceTo(_access[i].cir.c) < _access[i].cir.r){
                    logs << ERR << _path.path[_path.path_len - 1].p1.distanceTo(_access[i].cir.c);
                    continue;
                }
                else{
                    Point2D<float> pt = _path.path[_path.path_len - 1].p1;
                    Circle2D<float> cir(_access[i].cir.c.x, _access[i].cir.c.y, _access[i].cir.r);
                    pt = cir.projectSup(pt, 0.1);

                    if(checkPointInObs(pt, obs)){
                        continue;
                    }
                    _path.path[_path.path_len - 1].p2 = pt;


                    for (unsigned int j = 0; j < _path.path_len - 1; j++) {
                        Circle2D<float> c(_path.path[j].obs.c.x, _path.path[j].obs.c.y, _path.path[j].obs.r);
                        Point2D<float> p1(_path.path[j].p2.x, _path.path[j].p2.y), p2(_path.path[j+1].p1.x, _path.path[j+1].p1.y);
                        _path.path[j].arc_len = c.arcLenght(p1, p2);
                        _path.path[j].seg_len = _path.path[j].p1.distanceTo(_path.path[j].p2);
                    }

                    _path.path[_path.path_len - 1].arc_len = 0;
                    _path.path[_path.path_len - 1].seg_len = _path.path[_path.path_len - 1].p1.distanceTo(_path.path[_path.path_len - 1].p2);

                    _dist = 0;
                    for (unsigned int j = 0; j < _path.path_len; j++) {
                        _dist += _path.path[j].seg_len + _path.path[j].arc_len;
                    }

                    Vector2D<float> v1(1,0), v2(cir.c, pt);
                    _access_select_angle = v1.angle(v2) + M_PI; //M_PI because reference inverse
                }
            }
            else if(_access[i].type == E_POINT){
                _access_select_angle = _access[i].pt.angle + M_PI; //M_PI because reference inverse
                _dist = path_loc.dist;
            }
            _access_select_angle += _access[i].delta;
            _access_select_angle = fmod(_access_select_angle, 2*M_PI);
            _access_select = _path.path[_path.path_len - 1].p2;
        }
    }

    obs[robot].c = {posRobot.x, posRobot.y}; // Restore the original position

    return _dist;
}

int Obj::updateDestPointOrient(paramObj par){
    unsigned int i;

    if(par.act.empty())
        return -1;

    for(i = 0 ; i < par.act.size() ; i++){ //TODO optimize for the moment the first find is used
        if( par.act[i].type == _typeAct){
                break;
        }
    }

    if(i == par.act.size()){
        _actuator_select = -1;
        return -1;
    }

     _access_select_angle += par.act[i].angle;
     _actuator_select = par.act[i].id;

    return 0;
}

float Obj::getDist() const{
    return _dist;
}

sPath_t const& Obj::getPath() const{
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
        case E_OBJ_STARTING_ZONE:
            ratio = 10000000000;
            break;
        case E_SPOT3:
        case E_SPOT2:
            ratio = 1/_dist * 10000 ;
            logs << WAR << "ratio="<< ratio;
            break;
        case E_LIGHT:
            ratio = 1/_dist * 10000 ;
            logs << WAR << "ratio="<< ratio;
            break;
        case E_CUP:
            logs << ERR << millis();
            logs << ERR << start_time;
            ratio = 1/_dist * 10000 - ((int)millis() - (int)start_time)/1000;
            ratio = ratio>0?ratio:1;
            logs << ERR << "ratio_cup="<< ratio;
            break;
        case E_CLAP:
        case E_SPOT:
        default:
            ratio = 1/_dist * 10000; //TODO
            logs << WAR << "ratio="<< ratio;
            break;
    }
    return ratio * (1. - _done);
}


void Obj::print(){
    logs << INFO << fixed << setprecision(2) << "type : " << objType() << " : " << objState();
    if(_state == ACTIVE){
        if(_actuator_select == -1 && _dist > 0)
            logs << " : no actuator available";
        else if(_dist > 0)
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
