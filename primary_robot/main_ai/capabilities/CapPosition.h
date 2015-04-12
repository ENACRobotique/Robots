/*
 * CapPosition.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITY_POS_H_
#define CAPABILITY_POS_H_

#include <Capability.h>
#include "GeometryTools.h"
#include "tools.h"
#include "a_star_tools.h"

class CapPosition : public Capability{
    public:
        CapPosition(Robot* rob_init, iABObs_t iobs_init) : Capability(rob_init), iobs(iobs_init){};
        ~CapPosition(){};

        Point2D<float> getLastPosXY(){
            if(robot->cap[eCap::PROP] || robot->cap[eCap::BEACON]) //TODO Add video if use for positioning
                return statuses.getLastPosXY(robot->el);
            //if simu (example : secondary)

            //else impossible robot has NOT POSITION capability
            return {0, 0};
        }

        iABObs_t getIobs(){
            return iobs;
        }

    private:
        iABObs_t iobs; //obstacle associate to the robot

};

#endif /* CAPABILITY_POS_H_ */
