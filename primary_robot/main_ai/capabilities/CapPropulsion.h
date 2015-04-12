/*
 * CapPropulsion.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPPROPULSION_H_
#define CAPABILITIES_CAPPROPULSION_H_

#include <Capability.h>

class CapPropulsion : public Capability {
    public:
        CapPropulsion(Robot* robot_init, bn_Address address_init) : Capability(robot_init), address(address_init){};
        ~CapPropulsion(){};


        bn_Address getAddress(){
            return address;
        }

    private:
        bn_Address address;
};

#endif /* CAPABILITIES_CAPPROPULSION_H_ */
