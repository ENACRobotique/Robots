/*
 * CapSlave.h
 *
 *  Created on: 18 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPSLAVE_H_
#define CAPABILITIES_CAPSLAVE_H_

#include <vector>

#include <Capability.h>
#include "obj.h"
#include "communications.h"

#define INIT_POS_SLAVE_X 150
#define INIT_POS_SLAVE_Y 80
#define INIT_ANGLE_SLAVE -M_PI_2


class CapSlave: public Capability {
    public:
        CapSlave(Robot* robot_init) :
            Capability(robot_init) {
        }
        ~CapSlave() {
        }
        void setup(){
            Point2D<float> pt = {INIT_POS_SLAVE_X, INIT_POS_SLAVE_Y};
            sendSetPosPrimary(pt, INIT_ANGLE_SLAVE, 10.*10., 10.*10., 0., M_PI*M_PI/(10.*10.));
        }

        int loop(){
            Point2D<float> goal;
            CapPropulsion* capProp = dynamic_cast<CapPropulsion*> (robot->caps[eCap::PROP]);
            if (inbox.lastGoal(goal)) {
                logs << INFO << "New goal available";
                path.go2Point(goal, false, robot->env->obs, capProp->getPropType()==HOLO?true:false);
            }
            return 1;
        }
        void initObjective();

};



#endif /* CAPABILITIES_CAPSLAVE_H_ */
