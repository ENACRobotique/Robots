/*
 * \file performance.cpp
 * \author Yoann Solana
 * \brief: Functions that allow:
 *  to measure some remarkable values like
* 		_ the number of frame
* 		_ FPS: the number of frame per second
 */

#include "performance.h"


/**
 * \brief This function allow to measure the number of frame and the FPS
 * It is needed to call this function two time, first time at the start of the
 * measure and the second time at the end of the measure
 * \param ePerf state
 * \param sPerf& sPerfIn
 */
void cmptPerfFrame(ePerf state, sPerf& sPerfIn){
	if(state == StartPerf){
		// Get the initial time
		gettimeofday(&sPerfIn.tvStart, NULL);
		sPerfIn.nbFrame = 0;
	}
	else if(state == EndPerf){
		// Get the final time
		gettimeofday(&sPerfIn.tvEnd, NULL);

		// Compute the delta time
		int delta_us = cmptDeltaT_us(sPerfIn.tvStart, sPerfIn.tvEnd);

		sPerfIn.fps = 1000000./((float)delta_us);
		// Convert in second and display
		printf("PERF: Frame n°%i, %.2fFPS\n", sPerfIn.nbFrame++, sPerfIn.fps);
	}
}


/**
 * \brief Compute the difference of time in microsecond between the
 * input parameters
 * \param struct timeval tInit
 * \param struct timeval tEnd
 * \return time in microsecond
 */
int cmptDeltaT_us(struct timeval tInit, struct timeval tEnd){
	return ((tEnd.tv_sec - tInit.tv_sec)*1000000
			+ tEnd.tv_usec - tInit.tv_usec);
}
