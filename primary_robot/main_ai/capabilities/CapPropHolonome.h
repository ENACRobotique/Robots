/*
 * CapPropAxle.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPPROPHOLONOME_H_
#define CAPABILITIES_CAPPROPHOLONOME_H_

#include "CapPropulsion.h"

class CapPropHolonome : public CapPropulsion {
    public:
        CapPropHolonome(Robot* robot_init, bn_Address address_init) : CapPropulsion(robot_init, address_init){};
        ~CapPropHolonome(){};


};



#endif /* CAPABILITIES_CAPPROPHOLONOME_H_ */
