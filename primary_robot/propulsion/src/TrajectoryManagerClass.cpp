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
extern "C" {
	#include "utils.h"
}
#include <math.h>

TrajectoryManagerClass TrajectoryManager = TrajectoryManagerClass();

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
	if(_readIndex == _writeIndex) {		//no more points to read
		Serial.println("This is the end");
		return;
	}

	Point3D nextPoint = _objectives[_readIndex];
	double dx = nextPoint.getX() - Odometry.getPosX();
	double dy = nextPoint.getY() - Odometry.getPosY();
	double value;
	Serial.println("trajectoryStep : ");
	Serial.println(_trajectoryStep);
	switch (_trajectoryStep){
		case Stop:
			break;
		case InitialRotationStep:
			value = atan2(dy, dx) - Odometry.getThetaRad();
			value = constrainAngle(value);
			//TODO : tourner au minimum (vers la gauche ou la droite)
			Motors.computeParameters(value, Rotation);
			_trajectoryStep = CruiseStep;

			break;
		case CruiseStep:
			value = sqrt(pow(dx, 2) + pow (dy, 2));
			Motors.computeParameters(value, Straight);
			_trajectoryStep = FinalRotationStep;

			break;
		case FinalRotationStep:
			if (nextPoint.careAboutTheta()){
				value = nextPoint.getTheta() - Odometry.getThetaRad();
				value = constrainAngle(value);
				Motors.computeParameters(value, Rotation);
			}

			reached_point(_trajectoriesId[_trajReadIndex], _pointId);
			if (_pointId == _trajectoriesLength[_readIndex] - 1){ //Trajectoire finie
				_trajReadIndex = (_trajReadIndex + 1)%NB_POINTS_MAX;
				_pointId = 0;
			} else {
			_pointId++;
			}

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
	msg.x  = Odometry.getPosX();
	msg.y = Odometry.getPosY();
	msg.theta = Odometry.getThetaRad();
	msg.point_id = pointId;
	message_send(msg);
}

void TrajectoryManagerClass::addTrajectoryInfo(int trajId, int trajLength){
	_trajectoriesId[_trajWriteIndex] = trajId;
	_trajectoriesLength[_trajWriteIndex] = trajLength;
	_trajWriteIndex = (_trajWriteIndex + 1)%NB_POINTS_MAX;
}
