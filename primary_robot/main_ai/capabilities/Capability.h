/*
 * Capability.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITY_H_
#define CAPABILITY_H_

#include "messages-elements.h"
#include "Robot.h"


class Capability {
    public:
        Capability(Robot* robot_init) : robot(robot_init){}
        virtual ~Capability(){}

    protected:
        Robot* robot;
};

#endif /* CAPABILITY_H_ */
