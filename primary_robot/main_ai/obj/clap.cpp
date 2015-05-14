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

Point2D<float> obsClap[6]={{32., R_ROBOT}, {62., R_ROBOT}, {92., R_ROBOT}, {208. , R_ROBOT}, {238., R_ROBOT}, {268., R_ROBOT}} ;

Clap::Clap(unsigned int num) : Obj(E_CLAP, ActuatorType::POP_CORN_LOADER, true), _num(num){

    if(num > 6){
        logs << ERR << "Num too big";
    }
    sObjEntry_t objEP;
    objEP.type = E_POINT;
    objEP.pt.p = obsClap[num];
    objEP.radius = 8.;

    objEP.pt.angle = M_PI_2;

    _access.push_back(objEP);
}

Clap::~Clap() {
    // TODO Auto-generated destructor stub
}

int Clap::loopObj(std::vector<astar::sObs_t>&, std::vector<uint8_t>&, vector<Obj*>&, std::vector<Actuator>&){
    _state = FINISH;
    return 0;
};



