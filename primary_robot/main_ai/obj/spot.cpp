/*
 * clap.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#include "spot.h"

#include <iostream>
#include <cmath>

#include "math_types.h"
#include "types.h"
#include "tools.h"




Spot::Spot(unsigned int num) : Obj(E_SPOT), _num(num){

    if(num > 8){
        logs << ERR << "Num too big";
    }
    sObjPt_t objEP;
    if(color == GREEN)
        _numObs.push_back(START_STAND + num + 8);
    else
        _numObs.push_back(START_STAND + num);

    objEP.c = obs[_numObs.back()].c;
    objEP.radiusEP = 8.;
    objEP.angleEP = M_PI_2;

    _entryPoint.push_back(objEP);

}

Spot::~Spot() {
    // TODO Auto-generated destructor stub
}

int Spot::loopObj(){
    _active = false;
    return 0;
};



