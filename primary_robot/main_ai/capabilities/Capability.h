/*
 * Capability.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITY_H_
#define CAPABILITY_H_

#include "messages-elements.h"


class Robot;

class Capability {
    public:
        Capability(Robot* robot_init) : robot(robot_init){}
        ~Capability(){}

    protected:
        Robot* robot;
};

#endif /* CAPABILITY_H_ */
