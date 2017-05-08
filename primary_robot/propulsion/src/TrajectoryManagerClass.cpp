/*
 * TrajectoryManagerClass.cpp
 *
 *  Created on: 30 avr. 2017
 *      Author: fabien
 */

#include "TrajectoryManagerClass.h"
#include "OdometryController.h"
#include "MotorController.h"
extern "C" {
	#include "utils.h"
}
#include <math.h>

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

void TrajectoryManagerClass::readMessage(sMessageDown msg){
	eTypeDown msgType = msg.type;
	Point3D point;
	int ret;
	switch (msgType){
		case TRAJECTOIRE:
			for (int i=0; i < msg.traj.nb_trajectories; i++){
				if (i == msg.traj.nb_trajectories){ //Si c'est le dernier point,  careAboutTheta
					point = Point3D(msg.traj.element[i].x, msg.traj.element[i].y, msg.traj.theta_final);
				} else {
					point = Point3D(msg.traj.element[i].x, msg.traj.element[i].y);
				}
				addPoint(point, &ret);
				if (ret != 0){ // Buffer de trajectoire plein
					Serial.print("Tableau plein?");
				}
			}
			break;
		case STOP:
			//TODO : Stop
			break;
		case RESTART:
			//TODO : Restart
			break;
		case RECALAGE:
			//TODO : Recalage
			break;
		default:
			//this message should have been processed. Why are we here ?
			break;
	}
}
