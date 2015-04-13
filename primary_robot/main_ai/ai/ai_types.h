#ifndef _TYPE_IA_H
#define _TYPE_IA_H


#include <a_star_tools.h>
#include <main_ai_tools/path.h>
extern "C"{
#include <stdint.h>
#include "messages-interactions.h"
}
#include <vector>



#define COLOR_SIMU GREEN
#define SIMU 1 //modify network_cfg.h
#define PROG_TRAJ 1 //1 active
#define RESO_POS 2
#define NB_OBJ 16
#define END_MATCH 90000 //in ms
#define ERR_DIST 2.
#define NOMINAL_SPEED 20
#define LOW_SPEED 10
#define NB_MAX_PT_ZONE 10


typedef enum {
    COLOR_SELECTION, WAIT_STARTING_CORD, WAIT_START, WAIT_SECONDARY, GAME, SHUT_DOWN
} estate_t;

extern long _start_time;


#endif

