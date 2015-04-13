/*
 * CapPosition.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAP_POS_STATUSES_H_
#define CAP_POS_STATUSES_H_

#include <Capability.h>
#include "GeometryTools.h"
#include "tools.h"
#include "a_star_tools.h"

class CapPosStatuses : public CapPosition{
    public:
        CapPosStatuses(Robot* rob_init, iABObs_t iobs_init) : CapPosition(rob_init,iobs_init){};
        virtual ~CapPosStatuses(){};

        virtual Point2D<float> getLastPosXY(){
            return statuses.getLastPosXY(robot->el);
        }
};

#endif /* CAP_POS_STATUSES_H_ */
