/*
 * TrajectoryManagerClass.cpp
 *
 *  Created on: 30 avr. 2017
 *      Author: fabien
 */

#include "TrajectoryManagerClass.h"

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
