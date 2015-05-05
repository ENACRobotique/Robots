/*
 * obj_geometry_tools.h
 *
 *  Created on: 17 avr. 2014
 *      Author: seb
 */

#ifndef OBJ_GEOMETRY_TOOLS_H_
#define OBJ_GEOMETRY_TOOLS_H_

#include <a_star_tools.h>
#include <main_ai_tools/path.h>


void TransPt(sPt_t *traj, int x, int y);
void Rot90Pt(sPt_t *traj);
void SymPt(sPt_t *traj, int x, int y);

void TransElTraj(sTrajEl_t *traj, int x, int y);
void SymElTraj(sTrajEl_t *traj, int x, int y);
void Rot90Traj(sTrajEl_t *traj);

void projectPoint(sNum_t xp, sNum_t yp, sNum_t rc, sNum_t xc, sNum_t yc, sPt_t *point);

void tranOrg(sPt_t *pRef, sPt_t *p1, sPt_t *p2);


#endif /* OBJ_GEOMETRY_TOOLS_H_ */
