/*
 * obj_fruit.h
 *
 *  Created on: 17 avr. 2014
 *      Author: seb
 */

#ifndef OBJ_FRUIT_H_
#define OBJ_FRUIT_H_

#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "tools.h"
#include "millis.h"
#include "obj_types.h"
#include "obj_geometry_tools.h"
#include "obj_path_tools.h"
#include "obj_fct.h"
#include "math_ops.h"


int PATree(int num);
void updateEntryPointTree(void);
int getEntryPointTree(sPt_t *pt);
int sendInitTrajTree(iABObs_t obj);
void  checkFire(int num_obj, int idEntryTree);
void obj_tree(iABObs_t obj);
void obj_bac(iABObs_t obj);
sNum_t ratio_arbre(void);
sNum_t ratio_bac(void);


#endif /* OBJ_FRUIT_H_ */
