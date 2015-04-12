/*
 * CapPropAxle.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPPROPAXLE_H_
#define CAPABILITIES_CAPPROPAXLE_H_

#include "CapPropulsion.h"

class CapPropAxle : public CapPropulsion {
    public:
        CapPropAxle(Robot* robot_init, bn_Address address_init) : CapPropulsion(robot_init, address_init){};
        ~CapPropAxle(){};


};



#endif /* CAPABILITIES_CAPPROPAXLE_H_ */
