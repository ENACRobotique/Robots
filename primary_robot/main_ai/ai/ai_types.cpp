/*
 * type_ia.c
 *
 *  Created on: 20 févr. 2014
 *      Author: seb
 */

#include <ai_types.h>

long _start_time;
long last_time = 0;

sPt_t pt_select;
sPt_t _current_pos = { 0., 0. };

sNum_t speed = 0;
sNum_t theta_robot = 0.;
int starting_cord = 0;
int mode_switch = 0;
eColor_t color;
int current_obj = -1;

sPath_t path ;
uint8_t obs_updated[N] ;

