/*
 * messageHandlers.cpp
 *
 *  Created on: 8 mai 2017
 *      Author: fabien
 */

#include "messageHandlers.h"
#include "InputOutputs.h"
#include "TrajectoryManagerClass.h"
#include "OdometryController.h"

void handleMessage(sMessageDown msg) {
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
				TrajectoryManager.addPoint(point, &ret);
				if (ret != 0){ // Buffer de trajectoire plein
					Serial.print("Tableau plein?");
				}
			}
			break;
		case STOP:
			TrajectoryManager.stop();
			break;
		case RESTART:
			TrajectoryManager.resume();
			break;
		case RECALAGE:
			Odometry.setPosX(msg.recalage.x);
			Odometry.setPosY(msg.recalage.y);
			Odometry.setThetaRad(msg.recalage.theta);
			break;
		case LED:
			IOs.setLedColor(0,0,0);
			break;

	}
}
