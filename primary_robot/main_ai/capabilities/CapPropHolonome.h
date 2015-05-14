/*
 * CapPropAxle.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPPROPHOLONOME_H_
#define CAPABILITIES_CAPPROPHOLONOME_H_

#include <queue>

#include "CapPropulsion.h"

class CapPropHolonome: public CapPropulsion {
public:
    CapPropHolonome(Robot* robot_init, bn_Address address_init) :
            CapPropulsion(robot_init, address_init, ePropType::HOLO) {
    }
    virtual ~CapPropHolonome() {
    }

private:
    void sendTraj(const queue<sTrajEl_t>&) {
        // TODO
    }
    void sendTraj(const queue<sTrajOrientEl_t>&) {
        // TODO
    }

};

#endif /* CAPABILITIES_CAPPROPHOLONOME_H_ */
