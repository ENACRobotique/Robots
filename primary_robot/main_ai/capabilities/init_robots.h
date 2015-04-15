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

void setupRobots(bool simu_primary, bool holo_primary, eColor_t color_primary);
void loopRobots();

#endif /* CAPABILITIES_INIT_ROBOTS_H_ */
