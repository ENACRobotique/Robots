/*
 * CapPosSimuAdversary.h
 *
 *  Created on: 10 mai 2015
 *      Author: Sebastien Malissard
 */

#ifndef CAPABILITIES_CAPPOSSIMUADVERSARY_H_
#define CAPABILITIES_CAPPOSSIMUADVERSARY_H_

#include <Capability.h>
#include "CapPosition.h"
#include "GeometryTools.h"
#include "tools.h"
#include "CapTeam.h"

extern "C"{
#include "millis.h"
}

class CapPosSimuAdversary : public CapPosition{
    public:
        CapPosSimuAdversary(Robot* rob_init, astar::iABObs_t iobs_init) : CapPosition(rob_init, iobs_init), prevTime(0){
            robot->env->obs[iobs_init].active = 1;
        }
        virtual ~CapPosSimuAdversary(){}

        virtual Point2D<float> getLastPosXY(){
            if(prevTime == 0){
                CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);

                if(capTeam->getColor() == eColor_t::GREEN)
                    pos.x = 300 - 40;
                else
                    pos.x = 40;
                pos.y = 100;

                prevTime = millis();
            }
            else if(millis() - prevTime > 5000){
                prevTime = millis();
                pos.x = rand()%300;
                pos.y = rand()%200;
            }

            return pos;
        }
        virtual float getLastTheta(){
            return 0;
        }

    private:
        Point2D<float> pos;
        unsigned int prevTime;
};




#endif /* CAPABILITIES_CAPPOSSIMUADVERSARY_H_ */
