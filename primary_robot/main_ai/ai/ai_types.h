#ifndef _TYPE_IA_H
#define _TYPE_IA_H


#include <main_ai_tools/path.h>
#include "math_types.h"
#include "tools.h"
extern "C"{
#include <stdint.h>
#include "messages-interactions.h"
}

#define SPEED_SECONDARY 10 // (cm/s)

#define COLOR_SIMU GREEN
#define DEBUG 1
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
    COLOR_SELECTION, WAITING_POS, WAIT_STARTING_CORD, WAIT_START, WAIT_SECONDARY, GAME, SHUT_DOWN
} estate_t;

typedef enum {
    YELLOW, GREEN
} eColor_t;

typedef struct {
        estate_t next;
        sPt_t pos;
        sNum_t theta;
} sWaitPos;



extern uint8_t obs_updated[];

extern sPt_t _current_pos;
extern long _start_time;
extern long last_time;
extern sPt_t pt_select;
extern sNum_t speed;
extern sNum_t theta_robot;

extern int starting_cord;
extern int mode_switch;
extern eColor_t color;
extern int current_obj;

#endif

