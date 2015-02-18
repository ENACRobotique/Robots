/*
 * obj.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#include <geometry_tools.h>
#include <obj.h>
#include "math_ops.h"

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

Obj::Obj() :
         _type(0), _state(ACTIVE), _dist(-1), _time(-1), _point(-1), _EP(-1), _active(true), _done(0) {
}

/*Obj::Obj(eObj_t type, vector<unsigned int> numObs, vector<sObjPt_t> entryPoint) :
        _type(type), _state(ACTIVE), _numObs(numObs), _dist(-1), _time(-1), _point(-1), _EP(-1), _active(true), _done(0), _entryPoint(entryPoint) {
}*/

Obj::~Obj() {
    // TODO Auto-generated destructor stub
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

sNum_t Obj::updateDistance(sPt_t pos) {
    int g, m, n;
    sPath_t path_loc;

    _dist = -1;

    if ((n = testInObs(&pos)) != 0) {
        projectPoint(pos.x, pos.y, obs[n].r, obs[n].c.x, obs[n].c.y, &obs[0].c);
    }

    for (unsigned int i = 0; i < _entryPoint.size(); i++) {
        obs[N - 1].c = _entryPoint[i].c;
        obs[0].c = pos;

        //if robot non holonomic
        {
            updateEndTraj(_entryPoint[i].angleEP, &_entryPoint[i].c, _entryPoint[i].radiusEP); //plot the circle to arrive on objective

            if ((g = testInObs(&pos)) != 0) { //Projection if the robot is inside a "circle of end trajectory"
                projectPoint(pos.x, pos.y, obs[g].r, obs[g].c.x, obs[g].c.y, &obs[0].c);
                if ((m = testInObs(&obs[0].c)) != 0) { //Cas la projection se retrouve dans un obstacle après la premier projection
                    printf("Fix this projection inside an obstacle n_%d\n", m); //FIXME or no : investigate
                    continue;
                }
#if DEBUG
                printf("pos current after projection : x=%f, y=%f, obs x=%f, y=%f et r=%f\n", pos.x, pos.y, obs[g].c.x, obs[g].c.y, obs[g].r);
#endif
            }
        }

        //TODO déactivation d'un obstacle mobile si celui ci gene un point d'entré

        fill_tgts_lnk(); //TODO optimisation car uniquement la position de fin change dans la boucle
        a_star(A(0), A(N-1), &path_loc);

        //TODO reactivation des obstacles mobile deactiver precedement

        if (path_loc.dist == 0)
            printf("ATTENTION : A* retourne distance nul\n");
        else
            if (_dist > path_loc.dist || _dist == -1) {
                _dist = path_loc.dist;
                _EP = i;
            }
    }

    return _dist;
}

