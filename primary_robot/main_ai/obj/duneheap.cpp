/*
 * duneheap.cpp
 *
 *  Created on: 24 févr. 2016
 *      Author: guilhem
 */

#include <duneheap.h>

#include <iostream>
#include <cmath>

#include "types.h"
#include "tools.h"

extern "C"{
#include "millis.h"
}


DuneHeap::DuneHeap(unsigned int num) : Obj(E_DUNEHEAP, ActuatorType::SANDDOOR, true), _num(num), _time(0), stepLoc(DUNEHEAP_PREP){
    if(num > 2){
        logs << ERR << "Num too big";
    }

    _num_obs.push_back(START_DUNEHEAP + num);

    sObjEntry_t objEP;
    objEP.type = E_POINT;
    objEP.delta = 0;

    if (num == 0){
    	scratchPoint = {210., 170.};
    	scratchAngle = -M_PI/6;

    	swallowPoint = {200., 160.};
    	swallowAngle = -M_PI/6;

    	capturePoint = {205., 165.};
    	captureAngle = -M_PI/6;

    	objEP.pt.angle = -M_PI/6;
    	objEP.pt.p = {210., 150.};
    }else{
    	scratchPoint = {90., 170.};
    	scratchAngle = -M_PI/6;

    	swallowPoint = {100., 160.};
    	swallowAngle = -M_PI/6;

    	capturePoint = {95., 165.};
    	captureAngle = -M_PI/6;

    	objEP.pt.angle = -M_PI/6;
    	objEP.pt.p = {90., 150.};
    }

    _access.push_back(objEP);



}

DuneHeap::~DuneHeap() {
	// TODO Auto-generated destructor stub
}

void DuneHeap::initObj(paramObj){

}

int DuneHeap::loopObj(paramObj par){
	switch (stepLoc){
	case (DUNEHEAP_PREP) :
			servo.openDoor(DOOR_1_L);
			servo.openDoor(DOOR_1_R);

			path.go2PointOrient(scratchPoint, par.obs, scratchAngle);

			_time = millis();
			stepLoc = DUNEHEAP_CONTACT;
			break;

	case (DUNEHEAP_CONTACT):
		if(par.posRobot.distanceTo(scratchPoint) <= 2.){
			servo.grab4CubesDoor(0);
			_time=millis();
			stepLoc=DUNEHEAP_SCRAPE;
		}
		break;

	case (DUNEHEAP_SCRAPE):
		if (millis() - _time > 500){
			_time = millis();
			path.go2PointOrient(swallowPoint,par.obs, swallowAngle);
			stepLoc=DUNEHEAP_CAPTURE;
		}
		break;

	case (DUNEHEAP_CAPTURE):
		if (par.posRobot.distanceTo(swallowPoint) <= 2.){
			servo.openDoor(DOOR_1_L);
			servo.openDoor(DOOR_1_R);

			path.go2PointOrient(capturePoint, par.obs, captureAngle);

			stepLoc=DUNEHEAP_PREEND;
		}
		break;

	case (DUNEHEAP_PREEND):
		if (par.posRobot.distanceTo(capturePoint) <= 2.){
			servo.semicloseDoor(DOOR_1_L);
			servo.semicloseDoor(DOOR_1_R);

			_time = millis();
			stepLoc=DUNEHEAP_END;
		}
	 	break;

	case (DUNEHEAP_END):
		if (millis() - _time > 250.){
			_state =FINISH;
			return 0;
		}
		break;
	}
	return 1;
}
