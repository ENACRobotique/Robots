/*
 * clap.cpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#include "clap.h"

#include <iostream>
#include <cmath>

#include "math_types.h"
#include "types.h"


sPt_t obsClap[6]={{32., 30.}, {62., 30.}, {92., 30.}, {208. ,30.}, {238., 30.}, {268., 30.}} ;

Clap::Clap(unsigned int num) : Obj(E_CLAP), _num(num){

    if(num > 6){
        cerr << "[Error] [clap.cpp] Num too big" << endl;
    }
    sObjPt_t objEP;
    objEP.c = obsClap[num];
    objEP.radiusEP = 8.;
    objEP.angleEP = M_PI_2;

    _entryPoint.push_back(objEP);
}

Clap::~Clap() {
    // TODO Auto-generated destructor stub
}

int Clap::loopObj(){
    _active = false;
    return 0;
};



