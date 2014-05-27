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
int sendPosServo(eServos s, int16_t us, int16_t a);
int newSpeed(float speed);
int sendSeg(const sPt_t *p, const sVec_t *v);
int newSpeed(float speed);
void setPos(sPt_t *p, sNum_t theta);

#endif /* OBJ_COM_H_ */
