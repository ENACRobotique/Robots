/*
 * odometry.h
 *
 *  Created on: 6 mars 2017
 *      Author: fabien
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "Arduino.h"

double getX();
double getY();
double getTheta();
long getL();
long getIncLeft();
long getIncRight();
void setupOdometry();
extern IntervalTimer odometryTimer;



#endif /* ODOMETRY_H_ */
