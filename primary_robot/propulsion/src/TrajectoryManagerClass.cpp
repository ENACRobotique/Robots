/*
 * TrajectoryManagerClass.cpp
 *
 *  Created on: 30 avr. 2017
 *      Author: fabien
 */

#include "TrajectoryManagerClass.h"
#include "OdometryController.h"

TrajectoryManagerClass TrajectoryManager = TrajectoryManagerClass();

TrajectoryManagerClass::TrajectoryManagerClass() {
	_readIndex = 0;
	_writeIndex = 0;
	_trajectoryStep = InitialRotationStep;

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
	Point3D nextPoint = _objectives[_readIndex];
	double dx = Odometry.getPosX() - nextPoint.getX();
	double dy = Odometry.getPosY() - nextPoint.getY();
	switch (_trajectoryStep){
		case InitialRotationStep:
			double rotationAngle = atan2(dy, dx) - Odometry.getThetaRad();
			Motors.computeParameters(rotationAngle, Rotation);
			_trajectoryStep = CruiseStep;
			break;
		case CruiseStep:
			double translationLength = sqrt(pow(dx, 2) + pow (dy, 2));
			Motors.computeParameters(translationLength, Straight);
			_trajectoryStep = FinalRotationStep;
			break;
		case FinalRotationStep:
			if (nextPoint._careAboutTheta){
				double rotation = nextPoint.getTheta() - Odometry.getThetaRad();
			}
			_trajectoryStep = InitialRotationStep;
			_readIndex = (_readIndex + 1)%NB_POINTS_MAX;
	}
}
