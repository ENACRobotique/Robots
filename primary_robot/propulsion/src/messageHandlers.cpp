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
			//TODO : supprimer cette ligne si elle n'a plus d'impact (c'est à priori le cas)
			//TrajectoryManager.addTrajectoryInfo(msg.id, msg.traj.nb_trajectories);
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
			TrajectoryManager.stopRecalage();
			theta = (double)msg.recalage.theta / RAD_TO_UINT16;
			Odometry.setPosX(msg.recalage.x);
			Odometry.setPosY(msg.recalage.y);
			Odometry.setThetaRad(theta);
			break;
		case EMPTY_POINTS:
			TrajectoryManager.emptyPoints();
			break;
		case DO_RECALAGE:
			TrajectoryManager.doRecalage();
			break;
		case DO_REAR_RECALAGE:
			TrajectoryManager.doRearRecalage();
			break;
		case RESET:
			RESTART();
			break;
		case OPEN_TRAP:
			IOs.setServoPositionMs(SERVO_TRAP, GO_TRAP_OPEN);
			break;
		case CLOSE_TRAP:
			IOs.setServoPositionMs(SERVO_TRAP, GO_TRAP_CLOSE);
			break;
		case SORTER_BALL_1:
			IOs.setServoPositionMs(SERVO_SORTER, COLLECT_BALL_1);
			break;
		case SORTER_BALL_2:
			IOs.setServoPositionMs(SERVO_SORTER, COLLECT_BALL_2);
			break;
		case SORTER_UP:
			IOs.setServoPositionMs(SERVO_SORTER, GO_SORTER_UP);
			break;
		case CUTTER_OPEN:
			IOs.setServoPosition(SERVO_CUTTER, GO_CUTTER_OPEN);
			break;
		case CUTTER_CLOSE:
			IOs.setServoPosition(SERVO_CUTTER, GO_CUTTER_CLOSE);
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
