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
#include "dropCup.h"

extern "C"{
#include "millis.h"
}




Cup::Cup(unsigned int num, vector<astar::sObs_t>& obs) : Obj(E_CUP, ActuatorType::CUP, true), _num(num), _time(0){

    if(num > 4)
        logs << ERR << "Num too big";

    _num_obs.push_back(START_CUP + num);

    sObjEntry_t objEP;
    objEP.type = E_CIRCLE;
    objEP.delta = 0;
    objEP.cir.c = {obs[_num_obs.back()].c.x, obs[_num_obs.back()].c.y};
    objEP.cir.r = 5. + R_ROBOT;

    _access.push_back(objEP);

}

Cup::~Cup() {
    // TODO Auto-generated destructor stub
}

void Cup::initObj(Point2D<float> pos, vector<astar::sObs_t>& obs, vector<Obj*>&){
    Circle2D<float> cir(obs[_num_obs[0]].c.x, obs[_num_obs[0]].c.y, 10);
    Point2D<float> dest;

    dest = cir.project(pos);

    path.go2PointOrient(dest, obs, _access_select_angle);

    _time = millis();
}

int Cup::loopObj(const float&, std::vector<astar::sObs_t>&, std::vector<uint8_t>&,vector<Obj*>& listObj, std::vector<Actuator>& actuator){

    if(millis() - _time > 2000){
        for(Obj* i : listObj){
            if(i->type() == E_DROP_CUP && i->state() == WAIT_MES){
                i->state() = ACTIVE;
                break;
            }
        }

        for(Actuator& i : actuator){
            if(i.type == ActuatorType::CUP && i.id == _actuator_select){
                i.full = true;
                i.cupActuator.distributor = false; //to be sure
            }
        }
        _state = FINISH;
        return 0;
    }

    return 1;
};

