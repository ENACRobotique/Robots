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
            Actuator spot, cup;
            float spotAngle[2] = {M_PI/6, 5*M_PI/6};
            float cupAngle[3] = {0, 2*M_PI/3, 4*M_PI/3};
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
                _act.back().full = false;
                _act.back().angle = spotAngle[i];
                _act.back().elevator.ball = i==0?true:false;
                _act.back().elevator.number = 0;
            }

            for(unsigned int i = 0 ; i < 3 ; i++){
                _act.push_back(cup);
                _act.back().id = i;
                _act.back().type = ActuatorType::CUP;
                _act.back().full = false;
                _act.back().angle = cupAngle[i];
                _act.back().pos = cupPos[i];
                _act.back().cupActuator.distributor = false;
            }
        }

        std::vector<Actuator> _act;

};




#endif /* CAPABILITIES_CAPACTUATOR_H_ */
