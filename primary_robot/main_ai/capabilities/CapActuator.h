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

            _act.push_back(spot);
            _act.back().type = E_SPOT;
            for(unsigned int i = 0 ; i < 2 ; i++){
                _act.back().active.push_back(1);
                _act.back().angle.push_back(M_PI/3);
            }

            _act.push_back(cup);
            _act.back().type = E_CUP;
            for(unsigned int i = 0 ; i < 3 ; i++){
                _act.back().active.push_back(new bool(1));
                _act.back().angle.push_back(i*2*M_PI/3);
            }

            logs << DEBUG << "size=" << _act[0].active.size();
        }

        std::vector<Actuator> _act;

};




#endif /* CAPABILITIES_CAPACTUATOR_H_ */
