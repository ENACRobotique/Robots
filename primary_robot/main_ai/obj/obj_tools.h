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
void updateNoHaftTurn(sNum_t theta, sPt_t *pt);
void set_traj(sPath_t *p, iABObs_t l[], int nb);

int next_obj(void);
int metObj(int numObj);


#endif /* OBJ_OBJ_TOOLS_H_ */
