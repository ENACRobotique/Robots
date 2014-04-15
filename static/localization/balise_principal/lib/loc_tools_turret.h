/*
 * loc_tools_turret.h
 *
 *  Created on: 4 avr. 2014
 *      Author: quentin
 */

#ifndef LOC_TOOLS_TURRET_H_
#define LOC_TOOLS_TURRET_H_

#include "stdint.h"
#include "messages.h"

//#define DEBUG_LOC

/* Converts a time value to a angle in radian, based on the last few recorded turns of the turret
 * Argument :
 *  time : time (synchronized) at which the beacon has seen the laser passing
 * Return value :
 *  0 if correct and *ret is modified
 *  <0 is error.
 *
 */
int time2rad(uint32_t time, float *ret);

/* Makes the required computations and sends the message to IA with the measured position
 * Argument :
 *  pLoad : pointer to the data send by remote beacon
 *  origin : source adress of the pLoad data.
 * Return value :
 *  0 : everything went fine, *Pload can be cleared, the message to IA has been send
 *  <0 : something went wrong. Test return value to know if retries would be a good thing.
 */
int handleMeasurePayload(sMobileReportPayload *pLoad, bn_Address origin);

#endif /* LOC_TOOLS_TURRET_H_ */
