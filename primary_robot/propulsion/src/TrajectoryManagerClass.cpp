/*
 * TrajectoryManagerClass.cpp
 *
 *  Created on: 30 avr. 2017
 *      Author: fabien
 */

#include "TrajectoryManagerClass.h"
#include "OdometryController.h"
#include "MotorController.h"
#include "messages.h"
#include "params.h"
extern "C" {
	#include "utils.h"
}
#include <math.h>

TrajectoryManagerClass TrajectoryManager = TrajectoryManagerClass();

Point3D *lastPoint = NULL;

TrajectoryManagerClass::TrajectoryManagerClass() {
	_readIndex = 0;
	_writeIndex = 0;
	_trajectoryStep = InitialRotationStep;
	_trajReadIndex = 0;
	_trajWriteIndex = 0;
	_pointId = 0;
}

TrajectoryManagerClass::~TrajectoryManagerClass() {
	// TODO Auto-generated destructor stub
}

void TrajectoryManagerClass::addPoint(Point3D point, int * returnValue) {
	int nextWriteIndex = (_writeIndex + 1)%NB_POINTS_MAX;
	if(nextWriteIndex == _readIndex){
		if(returnValue != 0) {
			*returnValue = -1;
		}
		return;
	}

	_objectives[_writeIndex] = point;
	_writeIndex = nextWriteIndex;
	if(returnValue != 0) {
		*returnValue = 0;
	}
}

void TrajectoryManagerClass::readPoint(Point3D *point, int* returnValue) {
	if(!point) {
		return;
	}

	if( _readIndex == _writeIndex){
		if(returnValue != 0) {
			*returnValue = -1;
		}
		return;
	}

	*point = _objectives[_readIndex];
	_readIndex = (_readIndex + 1)%NB_POINTS_MAX;
}

void TrajectoryManagerClass::computeNextStep(){
	Serial.print("CNS : ");
	//Serial.print((long) lastPoint);
	if(lastPoint != NULL) {
		Serial.print(lastPoint->getX());
	}
	Serial.print(" ");
	Serial.println(_trajectoryStep);
	if (lastPoint != NULL && _trajectoryStep == InitialRotationStep){
		Serial.print("Traj id : ");
		Serial.println(lastPoint->getTrajId());
		reached_point(lastPoint->getTrajId(), lastPoint->getPointId());
		lastPoint = NULL;
	}
	if(_readIndex == _writeIndex) {		//no more points to read
		return;
	}

	//Point3D* nextPoint = &(_objectives[_readIndex]);
	Point3D* nextPoint = _objectives + _readIndex;
	double dx = nextPoint->getX() - Odometry.getPosX();
	double dy = nextPoint->getY() - Odometry.getPosY();
	int speed = nextPoint->getSpeed();
	double value;
	switch (_trajectoryStep){
		case Stop:
			break;
		case InitialRotationStep:
			value = atan2(dy, dx) - Odometry.getThetaRad();
			if(speed < 0) {
				value += PI;
			}
			value = constrainAngle(value);
			Motors.computeParameters(value, Rotation);
			_trajectoryStep = CruiseStep;

			break;
		case CruiseStep:
			value = sqrt(pow(dx, 2) + pow (dy, 2));
			if(speed < 0) {
				value = -value;
			}
			Motors.computeParameters(value, Straight, abs(speed));
			_trajectoryStep = FinalRotationStep;

			break;
		case FinalRotationStep:
			if (nextPoint->careAboutTheta()){
				value = nextPoint->getTheta() - Odometry.getThetaRad();
				value = constrainAngle(value);
				Motors.computeParameters(value, Rotation);
			}
			lastPoint = nextPoint;
			_trajectoryStep = InitialRotationStep;
			_readIndex = (_readIndex + 1)%NB_POINTS_MAX;
	}
}

void TrajectoryManagerClass::stop(){
	_trajectoryStep = Stop;
	Motors.computeParameters(0, Straight);
}

void TrajectoryManagerClass::resume(){
	_trajectoryStep = InitialRotationStep;
}

void TrajectoryManagerClass::emptyPoints() {
	Motors.computeParameters(0, Straight);
	_readIndex = 0;
	_writeIndex = 0;
	_trajReadIndex = 0;
	_trajWriteIndex = 0;
	_pointId = 0;
}

void TrajectoryManagerClass::reached_point(int trajId, int pointId){
	sMessageUp msg;
	msg.type = POINT_REACHED;
	msg.down_id = trajId;
	msg.x = (int) Odometry.getPosX();
	msg.y = (int) Odometry.getPosY();
	msg.theta = (int)(Odometry.getThetaRad() * RAD_TO_UINT16);
	msg.point_id = pointId;
	message_send(msg);
}

void TrajectoryManagerClass::addTrajectoryInfo(int trajId, int trajLength){
	_trajectoriesId[_trajWriteIndex] = trajId;
	_trajectoriesLength[_trajWriteIndex] = trajLength;
	_trajWriteIndex = (_trajWriteIndex + 1)%NB_POINTS_MAX;
}
