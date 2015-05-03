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

extern "C"{
#include "millis.h"
}




Cup::Cup(unsigned int num, vector<astar::sObs_t>& obs) : Obj(E_CUP), _num(num), _time(0){

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

void Cup::initObj(Point2D<float> pos, vector<astar::sObs_t>& obs){
    Circle2D<float> cir(obs[_num_obs[0]].c.x, obs[_num_obs[0]].c.y, 10);
    Point2D<float> dest;

    dest = cir.projecte(pos);

    path.go2Point(dest,true, obs, true); //FIXME angle actionneur

    _time = millis();
}

int Cup::loopObj(){

    if(millis() - _time > 2000){
        _state = FINISH;
        return 0;
    }

    return 1;
};

