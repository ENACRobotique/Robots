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

sGenericPos *getLastPGPosition(eElement el);

int received_new_generic_pos(sGenericPos *pos);

#endif /* OBJ_POSITION_H_ */
