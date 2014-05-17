/*
 * obj_com.h
 *
 *  Created on: 18 avr. 2014
 *      Author: seb
 */

#ifndef OBJ_COM_H_
#define OBJ_COM_H_

#include "unistd.h"

#include "tools.h"
#include "messages.h"
#include "roles.h"


void send_robot(sPath_t path);
void sendPosServo(eServos s, uint16_t us);
int newSpeed(float speed);
int sendSeg(const sPt_t *p, const sVec_t *v);

#endif /* OBJ_COM_H_ */
