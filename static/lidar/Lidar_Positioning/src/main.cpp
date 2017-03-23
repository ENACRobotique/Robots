/**
 \file    main.cpp
 \brief   Positioning on the table using a LIDAR.
 \author  {Fabien, Yoan}
 \date    19 mars 2016

This software is used for positioning a robot on a table using an ultra low-cost LIDAR.
It use the kwown fixed object of its environment to determine his position and orientation.


This is a licence-free software, it can be used by anyone who try to build a better world.
 */

#include <stdio.h>
#include <stdlib.h>
#include "iostream"
#include "ProcessLidarData.h"
#include "AcqLidar.h"
#include "time.h"
#include "ObjectRef.h"

extern"C"{
#include "millis.h"
}

/**  Minimum time between two computing of the position. */
#define COMPUTE_PERIOD 200

/**
 * Entry point of the program.
 */
int main()
{
	printf("début\n");
	unsigned int time_prev = millis();
	AcqLidar acqlidar = AcqLidar();
	initObjects();
	ProcessLidarData processor = ProcessLidarData(refObjects);
	if(acqlidar.init()) {
		perror("[LIDAR] Can't open serial.");
		return 1;
	}

	//PointOrient2D<int> lastPos(3000,1000,0);
	PointOrient2D<int> lastPos(2900,1020,0);

	while(true) {
		//get position by messages ->
		processor.setPosition(lastPos);
		acqlidar.updateData();
		if(millis() - time_prev > COMPUTE_PERIOD) {
			vector<PtLidar> data = acqlidar.getData();
			processor.makeGroups(data);
			//processor.process(data,lastPos);
			processor.correlateGroupsToObjects();
			cout << "aff" << endl;
			processor.affCorrelation();
			//processor.process();

		}
	}


	acqlidar.close();
	std::cout << "Fin du prgm !" <<std::endl;
}



/*!

  \brief Positioning on the table using a LIDAR.
  	  \mainpage main.cpp

    This software is used for positioning a robot on a table using an ultra low-cost LIDAR.
	It use the kwown fixed object of its environment to determine his position and orientation.


  \author   Fabien, Yoan
  \date     19 mars 2016


This is a licence-free software, it can be used by anyone who try to build a better world.
*/
