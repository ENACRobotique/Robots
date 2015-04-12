/*
 * CapIOProcess.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPIOPROCESS_H_
#define CAPABILITIES_CAPIOPROCESS_H_

#include <Capability.h>

class CapIOProcess : public Capability {
    public:
        CapIOProcess(Robot* robot_init) : Capability(robot_init){};
        ~CapIOProcess(){};


};




#endif /* CAPABILITIES_CAPIOPROCESS_H_ */
