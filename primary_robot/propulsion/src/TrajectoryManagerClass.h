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
	void addTrajectoryInfo(int trajId, int trajLentgh);
	void addPoint(Point3D point, int * returnValue);
	void emptyPoints();
	void readPoint(Point3D *point, int * returnValue);
	void reached_point(int trajId, int pointId);
	void computeNextStep();
	void testRecalage();
	void doRecalage();
	void stop();
	void resume();
private:
	TrajectoryStep _trajectoryStep;
	Point3D _objectives[NB_POINTS_MAX];
	int _readIndex;
	int _writeIndex;

	int _trajectoriesId[NB_POINTS_MAX]; // Numéros des messages contenant les trajectoires
	int _trajectoriesLength[NB_POINTS_MAX]; // Nombre de points des trajectoires
	int _trajReadIndex;
	int _trajWriteIndex;
	int _pointId; //Numéro du point dans sa trajectoire
	bool _recalageRunning;
};

extern TrajectoryManagerClass TrajectoryManager;

#endif /* TRAJECTORYMANAGERCLASS_H_ */
