/*
 * cup.cpp
 *
 *  Created on: 21 avr. 2015
 *      Author: Sebastien Malissard
 */


#include "cup.h"

#include <iostream>
#include <cmath>

#include "types.h"
#include "tools.h"




Cup::Cup(unsigned int num, vector<astar::sObs_t>& obs) : Obj(E_CUP), _num(num){

    if(num > 4)
        logs << ERR << "Num too big";

    _num_obs.push_back(START_CUP + num);

    sObjEntry_t objEP;
    objEP.type = E_CIRCLE;
    objEP.cir.c = {obs[_num_obs.back()].c.x, obs[_num_obs.back()].c.y};
    objEP.cir.r = 5. + R_ROBOT;

    _access.push_back(objEP);

}

Cup::~Cup() {
    // TODO Auto-generated destructor stub
}

int Cup::loopObj(){
    _state = FINISH;
    return 0;
};

