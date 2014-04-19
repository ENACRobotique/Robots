/*
 * obj_path_tools.h
 *
 *  Created on: 17 avr. 2014
 *      Author: seb
 */

#ifndef OBJ_PATH_TOOLS_H_
#define OBJ_PATH_TOOLS_H_

#include <math.h>

#include "tools.h"
#include "math_ops.h"


sNum_t seg_len(sPt_t *p1, sPt_t *p2);
void path_len(sTrajEl_t tab[], int size);
sNum_t arc_len2(sPt_t *p2_1, sPt_t *oc, sNum_t or, sPt_t *p2_3);
sNum_t distPath(sPath_t *path);

#endif /* OBJ_PATH_TOOLS_H_ */
