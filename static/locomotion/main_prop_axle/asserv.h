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
int new_orient_traj_el(sTrajOrientElRaw_t *toe);
int new_speed_setpoint(float speed);
int new_pos(sPosPayload *pos);
int send_pos();
void get_pos(s2DPosAtt *p, s2DPAUncert *p_u, unsigned int *p_t);
int new_asserv_step();
int show_stats();

#endif /* ASSERV_H_ */
