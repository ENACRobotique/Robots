/*
 * init_robots.cpp
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#include <vector>

#include "init_robots.h"

#include "network_cfg.h"

#include "Robot.h"
#include "CapPosition.h"
#include "CapPosSimuSecondary.h"
#include "CapPosStatuses.h"
#include "CapPropulsion.h"
#include "CapPropAxle.h"
#include "CapPropHolonome.h"
#include "CapAI.h"
#include "CapIO.h"


std::vector<Robot*> robots;


void initRobots(bool simu_primary, bool holo_primary){

    //Primary
    bn_Address primary_addr_prop = simu_primary?ADDRD1_MAIN_PROP_SIMU:ADDRI1_MAIN_PROP;

    robots.push_back(new Robot("", ELT_PRIMARY));

    robots.back()->caps[eCap::POS] = new CapPosStatuses(robots.back(), 1);
    if(holo_primary)
        robots.back()->caps[eCap::PROP] = new CapPropHolonome(robots.back(), primary_addr_prop);
    else
        robots.back()->caps[eCap::PROP] = new CapPropAxle(robots.back(), primary_addr_prop);
    robots.back()->caps[eCap::IO] = new CapIO(robots.back());
    robots.back()->caps[eCap::AI] = new CapAI(robots.back());


    //Secondary
    robots.push_back(new Robot("", ELT_SECONDARY));
    robots.back()->caps[eCap::POS] = new CapPosSimuSecondary(robots.back(), 2);
}


void loopRobots(){

    for(Robot* r : robots){
        if(CapPosition* capPos = dynamic_cast<CapPosition*> (r->caps[eCap::POS]))
            obs[capPos->getIobs()].c = {capPos->getLastPosXY().x, capPos->getLastPosXY().y};

        if(CapAI* capAI = dynamic_cast<CapAI*> (r->caps[eCap::AI]))
            capAI->loop();
    }
}
