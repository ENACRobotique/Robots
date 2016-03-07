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

Point2D<float> obsSandHeap[2]={
        { 35., 90.},
        { 265., 90.}
        };




SandHeap::SandHeap(unsigned int num) : Obj(E_SANDHEAP, ActuatorType::SANDDOOR, true), _num(num), _time(0), stepLoc(SAND_HEAP_PREP){

    if(num > 2)
        logs << ERR << "Num too big";

    _num_obs.push_back(START_HEAP + num);

    if (num == 0){
    	pointPutHeap.c = {130., 90.};
    	backPoint.c = {35., 90.};

    }
    else {
    	pointPutHeap.c = {170., 90.};
    	backPoint.c = {265., 90.};
    }

    backPoint.r = 5.;
    pointPutHeap.r = 5.;

    sObjEntry_t objEP;
    objEP.type = E_POINT;
    objEP.delta = 0;
    objEP.pt.p = obsSandHeap[num];
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
        case SAND_HEAP_PREP:
        	servo.closeDoor(DOOR_1_L);
        	servo.openDoor(DOOR_1_R);

            _time = millis();
            stepLoc = SAND_HEAP_PUSH;
            break;
        case SAND_HEAP_PUSH:
            if(millis() - _time > 500){
                _time = millis();
                destPoint = pointPutHeap.project(par.posRobot);
                path.go2PointOrient(destPoint, par.obs, _access_select_angle);
                stepLoc = SAND_HEAP_BACK;
            }
            break;
        case SAND_HEAP_BACK:
        	if (destPoint.distanceTo(par.posRobot) <= 5.){
        		destPoint = backPoint.project(par.posRobot);
        		path.go2PointOrient(destPoint, par.obs, _access_select_angle);
                _time = millis();
        		stepLoc = SAND_HEAP_END;
        	}
            break;

        case SAND_HEAP_END:
            if(millis() - _time > 500){
            	servo.closeDoor(DOOR_1_R);
                _state = FINISH;
                return 0;
            }
            break;
    }

    return 1;
};

