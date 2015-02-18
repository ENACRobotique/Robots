/*
 * tes.hpp
 *
 *  Created on: 16 févr. 2015
 *      Author: seb
 */

#ifndef MAIN_H_
#define MAIN_H_




#include "tools.h"

extern "C"{
#include <getopt.h>
#include <math.h>
#include "roles.h"
}

#include "a_star.h"
#include <ai_types.h>

extern sPath_t curr_path;
extern int curr_traj_extract_sid;
extern int last_tid;





#endif /* MAIN_H_ */
