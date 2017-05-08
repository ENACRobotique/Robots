/*
 * messageHandlers.cpp
 *
 *  Created on: 8 mai 2017
 *      Author: fabien
 */

#include "messageHandlers.h"
#include "InputOutputs.h"
#include "TrajectoryManagerClass.h"

void handleMessage(sMessageDown msg) {
	eTypeDown msgType = msg.type;
	int ret;
	switch (msgType){
		case TRAJECTOIRE:
		case STOP:
		case RESTART:
		case RECALAGE:
			TrajectoryManager.readMessage(msg);
			break;
		case LED:
			IOs.setLedColor(0,0,0);

	}
}
