/*
 * CapPropulsion.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPPROPULSION_H_
#define CAPABILITIES_CAPPROPULSION_H_

#include "roles.h"

#include "messages-locomotion.h"
#include "path.h"

#include "Capability.h"

enum ePropType {
    AXLE,
    HOLO
};

class CapPropulsion: public Capability {
protected:
    CapPropulsion(Robot* robot_init, bn_Address address_init, ePropType type) :
            Capability(robot_init), propType(type), role(ROLE_UNDEFINED) {
        switch (robot_init->el) {
        case ELT_PRIMARY:
            role = ROLE_PRIM_PROPULSION;
            break;
        case ELT_SECONDARY:
            role = ROLE_SEC_PROPULSION;
            break;
        default:
            break;
        }

        role_set_addr(role, address_init);
    }
    virtual ~CapPropulsion() {
    }

public:
    virtual void sendTraj(const queue<sTrajEl_t>& traj) = 0 ;
    virtual void sendTraj(const queue<sTrajOrientEl_t>& traj) = 0;

    ePropType getPropType() {
        return propType;
    }

private:
    ePropType propType;
    uint8_t role;
};

#endif /* CAPABILITIES_CAPPROPULSION_H_ */
