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




Cup::Cup(unsigned int num, vector<astar::sObs_t>& obs) : Obj(E_CUP, ActuatorType::CUP, true), _num(num), _time(0), stepLoc(CUP_OPEN_PINCE){

    if(num > 4)
        logs << ERR << "Num too big";

    _num_obs.push_back(START_CUP + num);

    sObjEntry_t objEP;
    objEP.type = E_CIRCLE;
    objEP.delta = 0;
    objEP.cir.c = {obs[_num_obs.back()].c.x, obs[_num_obs.back()].c.y};
    objEP.cir.r = 10. + R_ROBOT;

    _access.push_back(objEP);

}

Cup::~Cup() {
    // TODO Auto-generated destructor stub
}

void Cup::initObj(paramObj){

}

int Cup::loopObj(paramObj par){

    switch(stepLoc){
        case CUP_OPEN_PINCE :
            servo.unlockPince(par.act[_actuator_select].id);
            stepLoc = CUP_TRAJ1;
            break;
        case CUP_TRAJ1:
            {
                Circle2D<float> cir(par.obs[_num_obs[0]].c.x, par.obs[_num_obs[0]].c.y, 25.4);
                Point2D<float> dest;

                destPoint = cir.project(par.posRobot);

                path.go2PointOrient(destPoint, par.obs, _access_select_angle);

                stepLoc = CUP_WAIT_TRAJ1;
            }
            break;

        case CUP_WAIT_TRAJ1:
            if(par.posRobot.distanceTo(destPoint)){
                stepLoc = CUP_CLOSE;
            }
            break;

        case CUP_CLOSE:
            servo.inPince(par.act[_actuator_select].id);
            _time = millis();
            stepLoc = CUP_UP;
            break;

        case CUP_UP:
            if(millis() - _time > 500){
                servo.upPince(par.act[_actuator_select].id);
                _time = millis();
                stepLoc = CUP_END;
            }
            break;

        case CUP_END:
            if(millis() - _time > 500){
                for(Actuator& i : par.act){
                    if(i.type == ActuatorType::CUP && i.id == _actuator_select){
                        i.cupActuator.full = true;
                        i.cupActuator.distributor = false; //to be sure
                    }
                }
                _state = FINISH;
                return 0;
            }
            break;
    }

    return 1;
};

