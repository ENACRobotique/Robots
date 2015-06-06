/*
 * CapActuator.h
 *
 *  Created on: 3 mai 2015
 *      Author: seb
 */

#ifndef CAPABILITIES_CAPACTUATOR_H_
#define CAPABILITIES_CAPACTUATOR_H_

#include <vector>

#include <Capability.h>
#include "obj.h"

class CapActuator : public Capability{
    public:
        CapActuator(Robot* rob_init) : Capability(rob_init){}
        virtual ~CapActuator(){}

        vector<Actuator> getActuator(){
            return _act;
        }

        void setup(){
            Actuator spot, cup, popCornLoader, camera;
            float spotAngle[2] = {58.16*M_PI/180., -181.84*M_PI/180};
            float cupAngle[3] = {345.74*M_PI/180, 114.59*M_PI/180, -123.6*M_PI/180};
            float popCornLoaderAngle[] = {M_PI/2, 3*M_PI/2};

            Point2D<float> cupPos[3];
            cupPos[0].x = (R_ROBOT + 4.)*cos(cupAngle[0]);
            cupPos[0].y = (R_ROBOT + 4.)*sin(cupAngle[0]);
            cupPos[1].x = (R_ROBOT + 4.)*cos(cupAngle[1]);
            cupPos[1].y = (R_ROBOT + 4.)*sin(cupAngle[1]);
            cupPos[2].x = (R_ROBOT + 4.)*cos(cupAngle[2]);
            cupPos[2].y = (R_ROBOT + 4.)*sin(cupAngle[2]);

            for(unsigned int i = 0 ; i < 2 ; i++){
                _act.push_back(spot);
                _act.back().type = ActuatorType::ELEVATOR;
                _act.back().id = i;
                _act.back().elevator.full = false;
                _act.back().elevator.empty = true;
                _act.back().angle = spotAngle[i];
                //TODO _act.back().pos
                _act.back().elevator.ball = i==0?true:false; //if modify, change in objStatingZone
                _act.back().elevator.number = 0;
            }

            for(unsigned int i = 0 ; i < 3 ; i++){
                _act.push_back(cup);
                _act.back().id = i;
                _act.back().type = ActuatorType::CUP;
                _act.back().cupActuator.full = false;
                _act.back().angle = cupAngle[i];
                _act.back().pos = cupPos[i];
                _act.back().cupActuator.distributor = false;
            }

            for(unsigned int i = 0 ; i < 2 ; i++){
                _act.push_back(popCornLoader);
                _act.back().id = i;
                _act.back().type = ActuatorType::POP_CORN_LOADER;
                _act.back().angle = popCornLoaderAngle[i];
                //TODO _act.back().pos
                _act.back().cupActuator.distributor = false;
            }

            _act.push_back(camera);
            _act.back().id = 0;
            _act.back().type = ActuatorType::CAMERA;
            _act.back().angle = M_PI;
            _act.back().pos = {-20., 0.};
        }

        std::vector<Actuator> _act;

};




#endif /* CAPABILITIES_CAPACTUATOR_H_ */
