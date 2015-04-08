/*
 * init_robots.cpp
 *
 *  Created on: 30 mars 2015
 *      Author: Sébastien Malissard
 */

#include "init_robots.h"

std::vector<Robot*> listRobot;

Robot primary("", ELT_PRIMARY);

void init_robots(){
    primary.cap[eCap::POS] = new CapPosition();
    primary.cap[eCap::PROP] = new CapPropulsion();

}
