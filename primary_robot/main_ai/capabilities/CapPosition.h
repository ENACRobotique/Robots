/*
 * CapPosition.h
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITY_POS_H_
#define CAPABILITY_POS_H_

#include <capabilities/Capability.h>
#include "GeometryTools.h"
#include "tools.h"

class CapPosition : public Capability{
    public:
        CapPosition();
        ~CapPosition();

        Point2D<float> getLastPosXY(){
            return statuses.getLastPosXY(robot);
        }

};

#endif /* CAPABILITY_POS_H_ */
