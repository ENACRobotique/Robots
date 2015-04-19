/*
 * CapPosition.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAP_POS_SIMU_SECONDARY_H_
#define CAP_POS_SIMU_SECONDARY_H_

#include <Capability.h>
#include "CapPosition.h"
#include "GeometryTools.h"
#include "tools.h"
#include "a_star_tools.h"

#define SPEED_SECONDARY 10      // (cm/s)
#define START_DELAY     2000    // (ms)

class CapPosSimuSecondary : public CapPosition{
    public:
        CapPosSimuSecondary(Robot* rob_init, iABObs_t iobs_init) : CapPosition(rob_init, iobs_init){
            obs[iobs_init].active = 1;
        }
        virtual ~CapPosSimuSecondary(){}

        virtual Point2D<float> getLastPosXY();
};

#endif /* CAP_POS_SIMU_SECONDARY_H_ */
