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




Spot::Spot(unsigned int num) : Obj(E_SPOT), _num(num){

    if(num > 8){
        logs << ERR << "Num too big";
    }

    if(color == GREEN)
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

int Spot::loopObj(){
    _state = FINISH;
    obs[_num_obs.front()].active = 0;
    obs_updated[_num_obs.front()]++;
    return 0;
};



