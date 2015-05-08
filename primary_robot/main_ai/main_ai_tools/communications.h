/*
 * communications_tools.h
 *
 *  Created on: 22 févr. 2015
 *      Author: seb
 */

#ifndef COMMUNICATIONS_TOOLS_H_
#define COMMUNICATIONS_TOOLS_H_

#include <fstream>
#include <vector>
#include "GeometryTools.h"
#include "a_star_tools.h"

using namespace std;

//Send message
void sendPing();
int roleSetup(bool simu_ai, bool simu_prop);
void sendObsCfg(const int n, const int rRobot, const int xMin, const int xMax, const int yMin, const int yMax);
void sendObss(vector<astar::sObs_t>& obs, vector<uint8_t>& obs_updated);
int sendPosPrimary(Point2D<float>& p, float theta);
int sendSpeedPrimary(float speed);

//Receive message
void checkInbox(int verbose);
bool lastGoal(Point2D<float>& goal, bool get);
bool askObsCfg(bool get);

#endif /* COMMUNICATIONS_TOOLS_H_ */
