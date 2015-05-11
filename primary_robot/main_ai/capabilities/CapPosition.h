/*
 * CapPosition.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAP_POSITION_H_
#define CAP_POSITION_H_

#include <Capability.h>
#include "GeometryTools.h"
#include "tools.h"
#include "a_star_tools.h"

class CapPosition : public Capability{
    public:
        CapPosition(Robot* rob_init, astar::iABObs_t iobs_init) : Capability(rob_init), iobs(iobs_init){}
        virtual ~CapPosition(){}

        virtual Point2D<float> getLastPosXY() = 0;
        virtual float getLastTheta() = 0;

        astar::iABObs_t getIobs(){
            return iobs;
        }

    private:
        astar::iABObs_t iobs; //obstacle associate to the robot

};

#endif /* CAP_POSITION_H_ */
