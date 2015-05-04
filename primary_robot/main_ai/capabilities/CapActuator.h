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
            float cupAngle[3] = {0, M_PI/3, 2*M_PI/3};


            for(unsigned int i = 0 ; i < 2 ; i++){
                _act.push_back(spot);
                _act.back().type = E_SPOT;
                _act.back().full = false;
                _act.back().angle = spotAngle[i];
            }

            for(unsigned int i = 2 ; i < 5 ; i++){
                _act.push_back(cup);
                _act.back().type = E_CUP;
                _act.back().full = false;
                _act.back().angle = cupAngle[i-2];
            }
        }

        std::vector<Actuator> _act;

};




#endif /* CAPABILITIES_CAPACTUATOR_H_ */
