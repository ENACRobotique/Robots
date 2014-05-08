/*
 * obj.h
 *
 *  Created on: 17 mars 2014
 *      Author: ludo6431
 */

#ifndef OBJ_H_
#define OBJ_H_


#include <time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../global_errors.h"
#include "node_cfg.h"

#include "a_star.h"
#include "math_types.h"
#include "math_ops.h"
#include "tools.h"
#include "obj_types.h"
#include "obj_fct.h"
#include "millis.h"
#include "obj_fire.h"
#include "obj_fruit.h"
#include "obj_statuses.h"


void updateEndTraj(sNum_t theta, sPt_t *pt, sNum_t r);
void updateNoHaftTurn(sNum_t theta, sPt_t *pt);
sNum_t val_obj(int num);
int8_t next_obj (void);
void obj_step(void);
int obj_init(void);

#endif /* OBJ_H_ */
