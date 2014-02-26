/*
 * asserv.h
 *
 *  Created on: 26 févr. 2014
 *      Author: ludo6431
 */

#ifndef ASSERV_H_
#define ASSERV_H_

#include "messages.h"

int new_traj_el(sTrajElRaw_t *te);
int new_pos(sPosPayload *pos);
int new_asserv_step();
int send_pos();

#endif /* ASSERV_H_ */
