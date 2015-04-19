/*
 * CapPreparation.h
 *
 *  Created on: 18 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPPREPARATION_H_
#define CAPABILITIES_CAPPREPARATION_H_

#include "Capability.h"


class CapPreparation: public Capability {
    public:
        CapPreparation(Robot* robot_init) :
            Capability(robot_init) {
        }
        ~CapPreparation() {
        }

        virtual int loop() = 0;
};


#endif /* CAPABILITIES_CAPPREPARATION_H_ */
