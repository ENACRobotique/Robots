/*
 * obj_tools.hpp
 *
 *  Created on: 12 févr. 2015
 *      Author: seb
 */

#ifndef OBJ_OBJ_TOOLS_H_
#define OBJ_OBJ_TOOLS_H_

#include <vector>

#include "a_star.h"
#include "obj.h"


void loadingPath(sPath_t _path, int num = -1);
int checkCurrentPathLenght(sPath_t &path);
void updateEndTraj(float theta, Point2D<float> *pt, float r, vector<astar::sObs_t>& obs);
void updateNoHaftTurn(float theta, Point2D<float>& pt, vector<astar::sObs_t>& obs);
void set_traj(sPath_t *p, astar::iABObs_t l[], int nb);

int nextObj(const unsigned int start_time, const int robot, const bool axle, paramObj par) ;
int metObj(int numObj, paramObj par);


#endif /* OBJ_OBJ_TOOLS_H_ */
