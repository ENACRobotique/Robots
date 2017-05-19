/*
 * Point3D.cpp
 *
 *  Created on: 1 mai 2017
 *      Author: fabien
 */

#include "Point3D.h"

Point3D::Point3D() {
	_x = _y = _theta = 0;
	_careAboutTheta = false;

}

Point3D::~Point3D() {
	// TODO Auto-generated destructor stub
}

Point3D::Point3D(double x, double y) {
	_x = x;
	_y = y;
	_theta = 0;
	_careAboutTheta = false;
	_speed = 0;
}

Point3D::Point3D(double x, double y, double theta) {
	_x = x;
	_y = y;
	_theta = theta;
	_careAboutTheta = true;
	_speed = 0;
}

Point3D::Point3D(double x, double y, int speed) {
	_x = x;
	_y = y;
	_theta = 0;
	_careAboutTheta = false;
	_speed = speed;
}

Point3D::Point3D(double x, double y, double theta, int speed) {
	_x = x;
	_y = y;
	_theta = theta;
	_careAboutTheta = true;
	_speed = speed;
}
