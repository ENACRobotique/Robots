/*
 * TrajectoryManagerClass.h
 *
 *  Created on: 30 avr. 2017
 *      Author: fabien
 */

#ifndef TRAJECTORYMANAGERCLASS_H_
#define TRAJECTORYMANAGERCLASS_H_

#include "Point3D.h"
#include "messages.h"
#define NB_POINTS_MAX 20

typedef enum {
	Stop,
	InitialRotationStep,
	CruiseStep,
	FinalRotationStep
}TrajectoryStep;

class TrajectoryManagerClass {
public:
	TrajectoryManagerClass();
	virtual ~TrajectoryManagerClass();
	void addPoint(Point3D point, int * returnValue);
	void readPoint(Point3D *point, int * returnValue);
	void computeNextStep();
	void stop();
	void resume();
	void readMessage(sMessageDown msg);
private:
	TrajectoryStep _trajectoryStep;
	Point3D _objectives[NB_POINTS_MAX];
	int _readIndex;
	int _writeIndex;
};

extern TrajectoryManagerClass TrajectoryManager;

#endif /* TRAJECTORYMANAGERCLASS_H_ */
