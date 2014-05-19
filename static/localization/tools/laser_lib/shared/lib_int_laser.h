/*
 * lib_int_laser.h
 *
 *  Created on: 1 mai 2013
 *      Author: quentin
 */

#ifndef LIB_INT_LASER_H_
#define LIB_INT_LASER_H_

#include <stdint.h>

#ifdef ARCH_328P_ARDUINO
#include "arduino/lib_laser_arduino.h"
#endif


#define LASER_THICK_MIN     24    // in µs refined with measurement
#define LASER_THICK_MAX     600 // in µs refined with measurement
#define LASER_MAX_MISSED    3
#define LASER_DEBOUNCETIME  20  //measured

#define LAT_SHIFT 2 //in µs TODO : refine

//"return" structure for laserDetect
typedef struct {
    unsigned long deltaT;    // µs
    unsigned long date;      // local µs
    unsigned long thickness; // µs, thickness of the "small" impulsion
}ldStruct;

//"return" structure for periodicLaser
typedef struct {
	unsigned long deltaT;       // µs, delay between two laser small peaks
	unsigned long date;         // local µs, when was the laser recorded last
	unsigned long thickness;    // µs, thickness of the small laser peak /!\ thickness==0 <=> no laser detected
	unsigned long period;       // µs, MEASURED period (0 if not applicable).
	int precision;              // xxx TDB
	long int sureness;          // TBD
}plStruct;


//structure associated to a particular pair of sensors
typedef struct {
	volatile unsigned long buf[8];
	volatile int index;             // index of the last value recorded
	unsigned long prevCall;         // local µs,  the laserDetect will only consider values recorded between prevCall and the current time
	unsigned long lastDetect;       // last date at which a laser was detected on this interrupt (0 if the previous try to detect was unsuccessful)
	int stage;                      // used by periodiclaser to detect at which state is this pair of sensor
	unsigned long lat;              // µs, authorized latency in tracking mode : +-lat/2
	unsigned long prevTime;         // local µs, prevTime & nextTime : used by periodicLaser for its time measurements
	unsigned long timeInc;          // µs,  : increment of time after which there is something to do
	int missed;                     // number of missed detections
	int intNb;                      // intNb : nb of the interrupt
}bufStruct;


extern volatile unsigned long laser_period; //rotation period of the lasers
//extern unsigned long lastDetectTrack;
extern bufStruct buf0;                      // must be initialized with the last field at 0
extern bufStruct buf1;                      // must be initialized with the last field at 1


//declarations :
void laserIntInit(int irqnb);
void laserIntDeinit();


//laserDetect : check if the buffers have recorded a laser
//return a ldStruct if something was detected, 0 otherwise.
// /!\ do not call too often
ldStruct laserDetect(bufStruct *bs);

//function to call periodically, ensures acquisition and tracking of our laser beam
int periodicLaser(bufStruct *bs,plStruct *pRet);

uint32_t delta2dist(unsigned long delta, unsigned long period);

#endif /* LIB_INT_LASER_H_ */
