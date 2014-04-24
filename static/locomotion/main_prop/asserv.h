/*
 * asserv.h
 *
 *  Created on: 26 févr. 2014
 *      Author: ludo6431
 */

#ifndef ASSERV_H_
#define ASSERV_H_

#include "messages.h"

extern int x, y, theta; // robot position (I<<SHIFT), robot heading (I.rad<<SHIFT)

void asserv_init();
int new_traj_el(sTrajElRaw_t *te);
int new_speed_setpoint(float speed);
int new_pos(sPosPayload *pos);
int new_asserv_step();
int send_pos();
int show_stats();

#endif /* ASSERV_H_ */
