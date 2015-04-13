/*
 * CapPosition.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAP_POS_SIMU_SECONDARY_H_
#define CAP_POS_SIMU_SECONDARY_H_

#include <Capability.h>
#include "GeometryTools.h"
#include "tools.h"
#include "a_star_tools.h"

class CapPosSimuSecondary : public CapPosition{
    public:
        CapPosSimuSecondary(Robot* rob_init, iABObs_t iobs_init) : CapPosition(rob_init, iobs_init){};
        virtual ~CapPosSimuSecondary(){};

        virtual Point2D<float> getLastPosXY(){
            // TODO
            return Point2D<float>();
        }
};

#endif /* CAP_POS_SIMU_SECONDARY_H_ */
