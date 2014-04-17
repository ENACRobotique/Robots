/*
 * obj_fct.h
 *
 *  Created on: 29 mars 2014
 *      Author: seb
 */

#ifndef OBJ_FCT_H_
#define OBJ_FCT_H_


#include<stdlib.h>

#include "obj.h"
#include "obj_types.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

extern sTrajEl_t tabEl[];
extern sTrajEl_t tabEl2[];
extern sTrajEl_t tabEl3[];

extern void obj_tree(iABObs_t obj);
extern void obj_bac(iABObs_t obj);
extern void obj_fire(iABObs_t obj);
void updateEntryPointTree(void);

#endif /* OBJ_FCT_H_ */
