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
#include "CapPropulsion.h"
#include "CapPropAxle.h"
#include "CapPropHolonome.h"
#include "CapIOProcHMI.h"
#include "CapAI.h"


std::vector<Robot*> robots;


void init_robots(bool simu_primary){

    //Primary
    bool primary_holo = true; //TODO put in option
    bn_Address primary_addr_prop = simu_primary?ADDRD1_MAIN_PROP_SIMU:ADDRI1_MAIN_PROP;
    std::vector<eIhmElement> primary_ihm = {IHM_STARTING_CORD,IHM_MODE_SWICTH,IHM_LED};

    robots.push_back(new Robot("", ELT_PRIMARY));

    robots.back()->cap[eCap::POS] = new CapPosition(robots.back(), 1);
    if(primary_holo)
        robots.back()->cap[eCap::PROP] =  robots.back()->cap[eCap::HOLO] = new CapPropHolonome(robots.back(), primary_addr_prop);
    else
        robots.back()->cap[eCap::PROP] = robots.back()->cap[eCap::AXLE] = new CapPropAxle(robots.back(), primary_addr_prop);
    //FIXME change the default address of role PROP
    robots.back()->cap[eCap::IO_PROC] = robots.back()->cap[eCap::HMI] = new CapIOProcHMI(robots.back(), primary_ihm);
    robots.back()->cap[eCap::AI] = new CapAI(robots.back());


    //Secondary
    robots.push_back(new Robot("", ELT_SECONDARY));
    robots.back()->cap[eCap::POS] = new CapPosition(robots.back(), 2);
}
