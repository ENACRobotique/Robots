/*
 * ai_tools.h
 *
 *  Created on: 29 mars 2014
 *      Author: Sébastien Malissard
 */

#ifndef AI_TOOLS_H_
#define AI_TOOLS_H_

#include <vector>

#include "GeometryTools.h"
#include "a_star_tools.h"
#include "messages-elements.h"


#define RESO_POS 2
#define END_MATCH 90000 //in ms
#define ERR_DIST 2.
#define NOMINAL_SPEED 20
#define LOW_SPEED 10
#define NB_MAX_PT_ZONE 10

typedef struct {
    Point2D<float>  dest;
    float           angle;
    Point2D<float>  newPos;
    int             action; //0 nothing, 1 x, 2 y and 3 end
}SimpleTraj;

void printObsActive(std::vector<astar::sObs_t>& obs);
unsigned int checkPointInObs(const Point2D<float>& p, std::vector<astar::sObs_t>& obs);
unsigned int checkPointInLimitPlayground(const Point2D<float>& p, const float limit);
Point2D<float> projectPointInObs(const Point2D<float>& p, std::vector<astar::sObs_t>& obs);
Point2D<float> projectPointInLimitPlayground(const Point2D<float>& p,  const float limit);
int colissionDetection(const eElement& robot, const std::vector<astar::sObs_t>& pos);
void posPrimary(std::vector<astar::sObs_t>& obs);
void setStartingPosition(std::vector<SimpleTraj>& traj,const Point2D<float>& curPt, const float& curAngle, const Point2D<float>& destPt, const float& destAngle, const std::vector<Segment2D<float>>& robot, const std::vector<Segment2D<float>>& playground);

#endif /* AI_TOOLS_H_ */
