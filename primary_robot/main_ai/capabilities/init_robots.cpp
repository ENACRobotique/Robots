/*
 * init_robots.cpp
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#include "init_robots.h"

#include "network_cfg.h"

#include "Robot.h"
#include "CapPosition.h"
#include "CapPropulsion.h"

std::vector<Robot*> robots;


void init_robots(bool simu_primary){

    robots.push_back(new Robot("", ELT_PRIMARY));
    robots.back()->cap[eCap::POS] = new CapPosition(robots.back(), 1);
    if(simu_primary){
        robots.back()->cap[eCap::PROP] = new CapPropulsion(robots.back(), ADDRD1_MAIN_PROP_SIMU);
        //FIXME change the default address of role PROP
    }
    else
        robots.back()->cap[eCap::PROP] = new CapPropulsion(robots.back(), ADDRI1_MAIN_PROP);

    robots.push_back(new Robot("", ELT_SECONDARY));
    robots.back()->cap[eCap::POS] = new CapPosition(robots.back(), 2);
}
