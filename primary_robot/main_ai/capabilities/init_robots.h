/*
 * init_robots.h
 *
 *  Created on: 30 mars 2015
 *      Author: seb
 */

#ifndef CAPABILITIES_INIT_ROBOTS_H_
#define CAPABILITIES_INIT_ROBOTS_H_

#include <vector>

#include "Robot.h"

extern std::vector<Robot*> listRobot;

void setupRobots(bool primary_prop_simu, bool primary_prop_holo, bool primary_hmi_simu, eColor_t primary_color);
void loopRobots();

#endif /* CAPABILITIES_INIT_ROBOTS_H_ */
