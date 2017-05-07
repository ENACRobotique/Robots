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
	int speed;
	double theta;
	switch (msgType){
		case TRAJECTOIRE:
			TrajectoryManager.addTrajectoryInfo(msg.id, msg.traj.nb_trajectories);
			for (int i=0; i < msg.traj.nb_trajectories; i++){
				speed = (msg.traj.traj_speed - 127) * SPEED_COEFF;
				point = Point3D(msg.traj.element[i].x, msg.traj.element[i].y, speed, msg.id, i);
				if (i == msg.traj.nb_trajectories - 1){ //Dernier point
					theta = (double)msg.traj.theta_final / RAD_TO_UINT16;
					point.setTheta(theta);
				}
				TrajectoryManager.addPoint(point, &ret);
				if (ret != 0){ // Buffer de trajectoire plein
					Serial.print("Tableau plein?");
					//TODO : send POINTS_BUFFER_FULL message.
				}
			}
			/*Serial.print("Theta final : ");
			Serial.print(theta);
			point.setTheta(theta);
			Serial.print(" -> ");
			Serial.println(point.careAboutTheta());*/
			break;
		case STOP:
			TrajectoryManager.stop();
			break;
		case RESTART:
			TrajectoryManager.resume();
			break;
		case RECALAGE:
			theta = (double)msg.recalage.theta / RAD_TO_UINT16;
			Odometry.setPosX(msg.recalage.x);
			Odometry.setPosY(msg.recalage.y);
			Odometry.setThetaRad(theta);
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

void reportPosition(){
	sMessageUp msg;
	msg.type = POSITION;
	msg.down_id = 0;
	msg.x  = (int) Odometry.getPosX();
	msg.y = (int) Odometry.getPosY();
	msg.theta = (int) (Odometry.getThetaRad() * RAD_TO_UINT16);
	msg.point_id = 0;
	message_send(msg);
}
