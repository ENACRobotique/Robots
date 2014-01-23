/*
 * lib_wall.h
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#ifndef LIB_WALL_H_
#define LIB_WALL_H_

#define LEFT 1
#define RIGHT 0

void wallInitHard(int left_sensor, int right_sensor);
void periodicWall();
void wallSetVal(int side, int dist, int speed);
int diff(int pin_1 , int pin_2);
int endWall(int pin_front , int pin_back);

#endif /* LIB_WALL_H_ */
