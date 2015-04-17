#ifndef PERFORMANCE_H
#define PERFORMANCE_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <sys/time.h>


/**
 * \enum ePerf
 * \bref Contains flags to start or stop a measure
 */
typedef enum ePerf{
	StartPerf,
//	InterPerf,
	EndPerf
}ePerf;


/**
 * \struct sPerf
 * \brief Contains values to measure performances
 */
typedef struct sPerf{
	int nbFrame;
	float fps;
	struct timeval tvStart;
//	struct timeval tvInter;
	struct timeval tvEnd;
}sPerf;


void cmptPerfFrame(ePerf state, sPerf& sPerfIn);
int cmptDeltaT_us(struct timeval t1, struct timeval t2);


#endif  //PERFORMANCE_H
