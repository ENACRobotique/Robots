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

Obj::Obj() : _state(ACTIVE), _dist(-1), _time(-1), _point(-1), _EP(-1), _active(true), _done(0) {
}

Obj::Obj(eObj_t type) : _type(type), _state(ACTIVE), _dist(-1), _time(-1), _point(-1), _EP(-1), _active(true), _done(0) {
}

Obj::Obj(eObj_t type, vector<unsigned int> &numObs, vector<sObjPt_t> &entryPoint) :
        _type(type), _state(ACTIVE), _dist(-1), _time(-1), _point(-1), _EP(-1), _active(true), _done(0) {

    for(unsigned int i : numObs){
        _numObs.push_back(i);
    }

    for(sObjPt_t i : entryPoint){
        _entryPoint.push_back(i);
    }
}

Obj::~Obj() {
    // TODO Auto-generated destructor stub
}



void Obj::setEP(sObjPt_t &pt){
    _entryPoint.push_back(pt);
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

    if ((n = testInObs(&posRobot)) != 0) {
        projectPoint(posRobot.x, posRobot.y, obs[n].r, obs[n].c.x, obs[n].c.y, &obs[0].c);
        cout << "[INFO] [obj.cpp] Robot in obstacle" << endl;
    }

    for (unsigned int i = 0; i < _entryPoint.size(); i++) {
        obs[N - 1].c = _entryPoint[i].c;
        obs[0].c = posRobot;

#ifdef NON_HOLONOMIC
            updateEndTraj(_entryPoint[i].angleEP, &_entryPoint[i].c, _entryPoint[i].radiusEP); //write the circle to arrive on objective in obs

            if ((g = testInObs(&posRobot)) != 0) { //Projection if the robot is inside a "circle of end trajectory"
                projectPoint(posRobot.x, posRobot.y, obs[g].r, obs[g].c.x, obs[g].c.y, &obs[0].c);
                if ((m = testInObs(&obs[0].c)) != 0) { //Cas la projection se retrouve dans un obstacle après la premier projection
                    printf("Fix this projection inside an obstacle n_%d\n", m); //FIXME or no : investigate
                    continue;
                }
#if DEBUG
                printf("pos current after projection : x=%f, y=%f, obs x=%f, y=%f et r=%f\n", posRobot.x, posRobot.y, obs[g].c.x, obs[g].c.y, obs[g].r);
            }
#endif
#endif

        //TODO déactivation d'un obstacle mobile si celui ci gene un point d'entré

        fill_tgts_lnk(); //TODO optimisation car uniquement la position de fin change dans la boucle
        a_star(A(0), A(N-1), &path_loc);

        //TODO reactivation des obstacles mobile deactiver precedement

        if (path_loc.dist == 0)
            printf("[INFO] [obj.cpp] ATTENTION : A* retourne distance nul\n");
        else
            if (_dist > path_loc.dist || _dist == -1) {
                _dist = path_loc.dist;
                _EP = i;
                _path = path_loc;
            }
    }

    return _dist;
}


sNum_t Obj::value(void) {
    sNum_t ratio = 0.;

    if (_dist == 0) { //FIXME on est sur le point donc valeur max
        printf("Attention : distance nul\n"); //Erreur ou objectif atteint
        return (-1);
    }

    if (_time > (END_MATCH - (millis() - _start_time))) { //insufficient time remaining
        _active = false;
        return 0;
    }

    switch (_type) {
        case E_CLAP:
        case E_SPOT:
            ratio = _point / _time * 100; //TODO
            break;
        default:
            return (-1);
    }
    return ratio * (1. - _done);
}

sPt_t Obj::destPoint(){
    return _path.path[_path.path_len - 1].p2;
}

