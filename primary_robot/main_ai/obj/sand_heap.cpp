/*
 * sand_heap.cpp
 *
 *  Created on: 11 janv. 2016
 *      Author: guilhem
 */


#include "sand_heap.h"

#include <iostream>
#include <cmath>

#include "types.h"
#include "tools.h"

extern "C"{
#include "millis.h"
}

Point2D<float> obsClap[2]={
        { 35., 110.},
        { 265., 110.}
        };




SandHeap::SandHeap(unsigned int num) : Obj(E_SANDHEAP, ActuatorType::SANDDOOR, true), _num(num), _time(0), stepLoc(SAND_HEAP_PUSH){

    if(num > 2)
        logs << ERR << "Num too big";

    _num_obs.push_back(START_CUP + num);

    sObjEntry_t objEP;
    objEP.type = E_POINT;
    objEP.delta = 0;
    objEP.pt.p = obsClap[num];
    objEP.pt.angle = M_PI;

    _access.push_back(objEP);

}

SandHeap::~SandHeap() {
    // TODO Auto-generated destructor stub
}

void SandHeap::initObj(paramObj){

}

int SandHeap::loopObj(paramObj par){

    switch(stepLoc){
        case SAND_HEAP_PUSH:
        	servo.closeDoor(0);
        	servo.closeDoor(1);
            _time = millis();
            stepLoc = SAND_HEAP_BACK;
            break;
        case SAND_HEAP_BACK:
            if(millis() - _time > 500){
                _time = millis();
                Circle2D<float> cir(170., 110., 5.);
                destPoint = cir.project(par.posRobot);
                path.go2PointOrient(destPoint, par.obs, _access_select_angle);
                stepLoc = SAND_HEAP_END;
            }
            break;

        case SAND_HEAP_END:/*
            if(millis() - _time > 500){
                for(Actuator& i : par.act){
                    if(i.type == ActuatorType::CUP && i.id == _actuator_select){
                        i.cupActuator.full = true;
                        i.cupActuator.distributor = false; //to be sure
                    }
                }*/
                _state = FINISH;
                return 0;
            //}
            break;
    }

    return 1;
};

