/*
 * ai_tools.h
 *
 *  Created on: 29 mars 2014
 *      Author: Sébastien Malissard
 */

#ifndef AI_TOOLS_H_
#define AI_TOOLS_H_

#include "GeometryTools.h"

#define RESO_POS 2
#define END_MATCH 90000 //in ms
#define ERR_DIST 2.
#define NOMINAL_SPEED 20
#define LOW_SPEED 10
#define NB_MAX_PT_ZONE 10


void printObsActive();
int checkPointInObs(Point2D<float>& p);
void colissionDetection();
void posPrimary();

#endif /* AI_TOOLS_H_ */
