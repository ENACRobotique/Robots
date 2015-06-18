/*
 * clap.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#include "clap.h"

#include <iostream>
#include <cmath>

#include "types.h"
#include "tools.h"

#define ECART 20.

Point2D<float> obsClap[6]={
        { 15., ECART},
        { 45., ECART},
        { 75., ECART},
        {225., ECART},
        {255., ECART},
        {285., ECART}
        };

Clap::Clap(unsigned int num) : Obj(E_CLAP, ActuatorType::POP_CORN_LOADER, true), _num(num){
    sObjEntry_t objEP;

    switch(num){
        case 0:
        case 1:
        case 2:
            objEP.pt.angle = M_PI;
            objEP.delta = -M_PI/2;
            break;
        case 3:
        case 4:
        case 5:
            objEP.pt.angle = 0;
            objEP.delta = M_PI/2;
            break;
        default:
            logs << ERR << "Num too big";
    }

    objEP.type = E_POINT;
    objEP.pt.p = obsClap[num];
    objEP.radius = 8.;

    _access.push_back(objEP);
}

Clap::~Clap() {
    // TODO Auto-generated destructor stub
}

void Clap::initObj(paramObj par){
    _dest.x = par.obs[0].c.x - 15*cos(_access[0].pt.angle);
    _dest.y = ECART;

    path.go2PointOrient(_dest, par.obs, _access_select_angle);
    servo.openPopcornLoader(_actuator_select);

}

int Clap::loopObj(paramObj par){

    if(par.posRobot.distanceTo(_dest) < 2.){
        servo.closePopcornLoader(_actuator_select);
        _state = FINISH;
        return 0;
    }
    return 1;
};



