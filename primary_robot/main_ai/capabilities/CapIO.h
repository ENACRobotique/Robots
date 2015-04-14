/*
 * CapIOProcess.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPIO_H_
#define CAPABILITIES_CAPIO_H_

#include <Capability.h>

class CapIO : public Capability {
    public:
        CapIO(Robot* robot_init) : Capability(robot_init){};
        virtual ~CapIO(){};

// TODO setServo, getServo, getHMI, setHMI

        unsigned int getValue(const eIhmElement& el){
            return ihm.getValue(el);
        }
};


#endif /* CAPABILITIES_CAPIO_H_ */
