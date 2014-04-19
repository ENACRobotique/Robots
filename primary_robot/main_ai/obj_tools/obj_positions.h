/*
 * obj_position.h
 *
 *  Created on: 18 avr. 2014
 *      Author: ludo6431
 */

#ifndef OBJ_POSITION_H_
#define OBJ_POSITION_H_

// in µs
#define SAME_DATE_THRESHOLD (1000)
#define KEEP_OLD_THRESHOLD (5*1000*1000)
#define PROP_ANSWER_TIMEOUT (2*1000*1000)

sGenericPos *getLastPGPosition(eElement el);

int received_new_generic_pos(sGenericPos *pos);
void positions_maintenance();

#endif /* OBJ_POSITION_H_ */
