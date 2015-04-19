/*
 * ai_tools.h
 *
 *  Created on: 29 mars 2014
 *      Author: Sébastien Malissard
 */

#ifndef AI_TOOLS_H_
#define AI_TOOLS_H_

#include "GeometryTools.h"
#include "a_star_tools.h"
#include <vector>

#define RESO_POS 2
#define END_MATCH 90000 //in ms
#define ERR_DIST 2.
#define NOMINAL_SPEED 20
#define LOW_SPEED 10
#define NB_MAX_PT_ZONE 10


void printObsActive(std::vector<astar::sObs_t>& obs);
int checkPointInObs(const Point2D<float>& p, std::vector<astar::sObs_t>& obs);
Point2D<float> projectPointInObs(const Point2D<float>& p, std::vector<astar::sObs_t>& obs);
void colissionDetection();
void posPrimary(std::vector<astar::sObs_t>& obs);

#endif /* AI_TOOLS_H_ */
