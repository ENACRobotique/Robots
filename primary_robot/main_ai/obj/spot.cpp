/*
 * clap.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#include "spot.h"

#include <iostream>
#include <cmath>

#include "types.h"
#include "tools.h"




Spot::Spot(unsigned int num, eColor_t color, vector<astar::sObs_t>& obs) : Obj(E_SPOT, ActuatorType::ELEVATOR, true), _num(num), _color(color){

    if(num > 8){
        logs << ERR << "Num too big";
    }

    if(_color == GREEN)
        _num_obs.push_back(START_STAND + num + 8);
    else
        _num_obs.push_back(START_STAND + num);

    sObjEntry_t objEP;
    objEP.type = E_CIRCLE;
    objEP.cir.c = {obs[_num_obs.back()].c.x, obs[_num_obs.back()].c.y};
    objEP.cir.r = 3. + R_ROBOT;

    _access.push_back(objEP);

}

Spot::~Spot() {
    // TODO Auto-generated destructor stub
}

int Spot::loopObj(std::vector<astar::sObs_t>&, std::vector<uint8_t>&,vector<Obj*>&, std::vector<Actuator>&){
    _state = FINISH;
    return 0;
};



