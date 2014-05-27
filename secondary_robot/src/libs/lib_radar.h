/*
 * lib_radar.h
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#ifndef LIB_RADAR_H_
#define LIB_RADAR_H_

#include "Arduino.h"
#include "Servo.h"
#include "lib_us.h"

//defines
#define DEBUG_RADAR

#define RAD_TIMER_1 70//70    //time between call for mesure and mesure reading (cf sensor datasheet)
#define RAD_TIMER_2 70//0   //time between mesure reading and call for next mesure (cf servo sweeping speed)


#define RAD_POS_MIN 	0
#define RAD_POS_MAX 	0
#define RAD_POS_INC 	1  //must be a submultiple of RAD_POS_MAX-RAD_POS_MIN && must be different from zero (equal to 1 if no servo is usedà
#define RAD_NB_POS  	1
#define RAD_NB_PTS  	2
#define RAD_NB_SENSORS 	2

/* how are stored the values in C_rad and C_rad_limits (0° pointing toward the front of the robot, angles increasing clockwise):
old index  angle  direction
old 0      202,5  rear-left
old 1      157,5  rear-right
old 2      112,5  right-rear
old 3      67,5   right-front
old 4      22,5   front-right
old 5      337,5  front-left
old 6      292,5  left-front
old 7      247,5  left-rear*/
void radarSetLim(uint16_t[RAD_NB_POS]);

void radarInitServo(int);

//function to call periodically to refresh the values red by the radar
void radarRefresh();

//returns the shortest range measured
uint16_t radarCloser();

//returns the number of range measures shorter than the limit defined in C_rad_limits
//(for the direction assiciated to each measure).
int radarIntrusion();

#endif /* LIB_RADAR_H_ */
