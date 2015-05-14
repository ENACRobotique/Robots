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
#include "servoTools.h"

extern "C"{
#include "millis.h"
}

#define TIME_ELEVATOR_DOWN_UP       1000
#define TIME_ELEVATOR_LOCK_UNCLOK   500

Spot::Spot(unsigned int num, eColor_t color, vector<astar::sObs_t>& obs) : Obj(E_SPOT, ActuatorType::ELEVATOR, true),
        _num(num), _color(color), stepLoc(SPOT_TRAJ1), timePrev(0){

    if(num > 4){
        logs << ERR << "Num too big";
    }

    if(_color == GREEN)
        _num_obs.push_back(START_STAND + num + 8);
    else
        _num_obs.push_back(START_STAND + num);

    sObjEntry_t objEP;
    objEP.type = E_CIRCLE;
    objEP.delta = 0;
    objEP.cir.c = {obs[_num_obs.back()].c.x, obs[_num_obs.back()].c.y};
    objEP.cir.r = 3. + R_ROBOT;

    _access.push_back(objEP);

}

Spot::~Spot() {
    // TODO Auto-generated destructor stub
}

int Spot::loopObj(paramObj par){

    switch(stepLoc){
        case SPOT_TRAJ1:
            {
                Circle2D<float> cir(par.obs[_num_obs[0]].c.x, par.obs[_num_obs[0]].c.y, 5);
                destPoint = cir.project(par.posRobot);
                path.go2PointOrient(destPoint, par.obs, _access_select_angle);
                stepLoc = SPOT_WAIT_TRAJ1;
            }
            break;

        case SPOT_WAIT_TRAJ1:
            if(par.posRobot.distanceTo(destPoint) < 1.){
                stepLoc = SPOT_GET_STAND1;
            }
            break;

        case SPOT_GET_STAND1:
            if(getStand(par.act[_actuator_select].id)){
                stepLoc = SPOT_END;
            }
            break;

        case SPOT_END:
            par.act[_actuator_select].elevator.number++;
            par.act[_actuator_select].elevator.empty = false;

            if(par.act[_actuator_select].elevator.number == 4){
                par.act[_actuator_select].elevator.full = true;

            }
            _state = FINISH;
            return 0;
            break;
    }

    return 1;
};



