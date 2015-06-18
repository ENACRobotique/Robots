/*
 * communications_tools.h
 *
 *  Created on: 22 févr. 2015
 *      Author: seb
 */

#ifndef COMMUNICATIONS_TOOLS_H_
#define COMMUNICATIONS_TOOLS_H_

#include <a_star_tools.h>
#include <messages.h>
#include <NodesNetwork.h>
#include <Point2D.h>
#include <cstdint>
#include <string>
#include <vector>

using namespace std;

//Send message
void bnSendBlock(sMsg& msg, const string& txt);
void roleSendBlock(sMsg& msg, eRoleMsgClass mc, const string& txt);
void sendPing(NodesNetwork&);
int roleSetup(bool simu_ai, bool simu_prop, bool simu_beacons);
int syncSetup(bool turet);
void sendObsCfg(const int n, const int rRobot, const int xMin, const int xMax, const int yMin, const int yMax);
void sendObss(vector<astar::sObs_t>& obs, vector<uint8_t>& obs_updated);
int sendSetPosPrimary(const Point2D<float> &p, const float theta, const float p_a_var, const float p_b_var, const float p_a_angle, const float theta_var);
int sendMixPosPrimary(const Point2D<float> &p, const float theta, const float p_a_var, const float p_b_var, const float p_a_angle, const float theta_var);
int sendSpeedPrimary(float speed);

//Receive message
bool askObsCfg(bool get);

#endif /* COMMUNICATIONS_TOOLS_H_ */
