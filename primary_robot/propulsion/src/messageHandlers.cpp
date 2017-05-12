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
#include "params.h"

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
		case EMPTY_POINTS:
			TrajectoryManager.emptyPoints();
			break;
		case START_BALL_PICKER_MOTOR:
			IOs.setPickerSpeed(PICKER_SPEED);
			break;
		case STOP_BALL_PICKER_MOTOR:
			IOs.setPickerSpeed(0);
			break;
		case START_CANNON_MOTOR:
			IOs.setLauncherSpeed(LAUNCHER_SPEED);
			break;
		case STOP_CANNON_MOTOR:
			IOs.setLauncherSpeed(0);
			break;
		case OPEN_CANNON_BARRIER:
			IOs.setServoPosition(SERVO_CANNON_BARRIER, CANNON_BARRIER_OPENED);
			break;
		case CLOSE_CANNON_BARRIER:
			IOs.setServoPosition(SERVO_CANNON_BARRIER, CANNON_BARRIER_CLOSED);
			break;
		case OPEN_ROCKET_LAUNCHER:
			IOs.setServoPosition(SERVO_ROCKET, ROCKET_LAUNCH);
			break;
		case LOCK_ROCKET_LAUNCHER:
			IOs.setServoPosition(SERVO_ROCKET, ROCKET_IDLE);
			break;
	}
}
