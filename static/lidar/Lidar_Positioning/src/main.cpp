/*
 * main.c
 *
 *  Created on: 10 févr. 2016
 *      Author: fabien
 */

#include <stdio.h>
#include <stdlib.h>
#include "iostream"
#include "ProcessLidarData.h"
#include "AcqLidar.h"
#include "time.h"

extern"C"{
#include "millis.h"
}

#define COMPUTE_PERIOD 200


int main()
{
	printf("début\n");
	unsigned int time_prev = millis();
	AcqLidar acqlidar = AcqLidar();
	ProcessLidarData processor = ProcessLidarData();
	acqlidar.init();
	PointOrient2D<int> lastPos;

	while(true) {
		if(millis() - time_prev > COMPUTE_PERIOD) {
			vector<PtLidar> data = acqlidar.getData();
			processor.process(data,lastPos);
		}
	}


	acqlidar.close();
	std::cout << "Fin du prgm !" <<std::endl;
}

