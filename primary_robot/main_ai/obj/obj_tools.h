/*
 * obj_tools.hpp
 *
 *  Created on: 12 févr. 2015
 *      Author: seb
 */

#ifndef OBJ_OBJ_TOOLS_H_
#define OBJ_OBJ_TOOLS_H_

#include <a_star_tools.h>

void loadingPath(sPath_t _path, int num = -1);
int checkCurrentPathLenght(sPath_t &path);
void updateEndTraj(float theta, Point2D<float> *pt, float r);
void updateNoHaftTurn(float theta, Point2D<float>& pt);
void set_traj(sPath_t *p, iABObs_t l[], int nb);

int next_obj(const unsigned int start_time);
int metObj(int numObj);


#endif /* OBJ_OBJ_TOOLS_H_ */
