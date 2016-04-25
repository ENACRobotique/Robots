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
            Actuator door, grip, camera;
//#error "Actuator angle and position to be defined !!!"
            float doorAngle[2] = {-181.84*M_PI/180, 180*M_PI/180};
            float gripAngle = 90.;

            Point2D<float> doorPos[2];
            doorPos[0].x = (R_ROBOT + 4.)*cos(doorAngle[0]);
            doorPos[0].y = (R_ROBOT + 4.)*sin(doorAngle[0]);
            doorPos[1].x = (R_ROBOT + 4.)*cos(doorAngle[1]);
            doorPos[1].y = (R_ROBOT + 4.)*sin(doorAngle[1]);

            Point2D<float> gripPos;
            gripPos.x = 2.;
            gripPos.y = 3.;

            for(unsigned int i = 0 ; i < 2 ; i++){
                _act.push_back(door);
                _act.back().type = ActuatorType::SANDDOOR;
                _act.back().id = i;
                _act.back().doors.cone_number = 0; //if modify, change in objStatingZone;
                _act.back().doors.cube_number =0;
                _act.back().doors.cylinder_number = 0;
                _act.back().angle = doorAngle[i];
                //TODO _act.back().pos
            }

            _act.push_back(grip);
            _act.back().id = 0;
            _act.back().type = ActuatorType::SANDGRIP;
            _act.back().grip.full = false;
            _act.back().angle = gripAngle;
            _act.back().pos = gripPos;

            _act.push_back(camera);
            _act.back().id = 0;
            _act.back().type = ActuatorType::CAMERA;
            _act.back().angle = M_PI;
            _act.back().pos = {-20., 0.};

        }



        std::vector<Actuator> _act;

};




#endif /* CAPABILITIES_CAPACTUATOR_H_ */
