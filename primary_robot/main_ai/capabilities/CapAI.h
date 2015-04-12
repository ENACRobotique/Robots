/*
 * CapAI.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sebastien Malissard
 */

#ifndef CAPABILITIES_CAPAI_H_
#define CAPABILITIES_CAPAI_H_

#include <Capability.h>

class CapAI : public Capability {
    public:
        CapAI(Robot* robot_init) : Capability(robot_init){};
        ~CapAI(){};

        void loop(){};

};



#endif /* CAPABILITIES_CAPAI_H_ */
