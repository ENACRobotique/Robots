/*
 * Capability.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITY_H_
#define CAPABILITY_H_

#include "messages-elements.h"

class Capability {
    public:
        Capability(eElement robot_init) : robot(robot_init){}
        ~Capability(){}

    protected:
        eElement robot;
};

#endif /* CAPABILITY_H_ */
