/*
 * lib_int_laser.h
 *
 *  Created on: 1 mai 2013
 *      Author: quentin
 */

#ifndef LIB_INT_LASER_H_
#define LIB_INT_LASER_H_



//"return" structure for detectLaser
typedef struct {
    unsigned long deltaT;
    unsigned long date;
    unsigned long thickness; //thickness of the "small" impulsion
}ldStruct;

//"return" structure for periodicLaser
typedef struct {
	unsigned long deltaT;
	unsigned long date;
	unsigned long thickness;
	int precision;
	int sureness;
}plStruct;


//structure associated to a particular pair of sensors
typedef struct {
	volatile unsigned long buf[8];
	//index of the last value recorded
	volatile int index;
	// the laserDetect will only consider values recorded between prevCall and the current time
	unsigned long prevCall;
	//used by periodiclaser to detect at which state is this pair of sensor
	int stage;
	// authorized latency in tracking mode : +-lat/2 (µs)
	unsigned long lat;
	// prevTime & nextTime : used by periodicLaser for its time measurements (µs)
	unsigned long prevTime;
	//nextTime : in what time there is something to do (µs)
	unsigned long nextTime;
	//intNb : nb of the interrupt
	int intNb;
}bufStruct;


extern unsigned long laser_period; //rotation period of the lasers
//extern unsigned long lastDetect;
extern bufStruct buf0;
extern bufStruct buf1;


//déclarations :
void laserIntInit(int irqnb);
void laserIntDeinit();
void laserIntHand0();
void laserIntHand1();

//laserDetect : check if the buffers have recorded a laser
//return a dtdStruct if something was detected, 0 otherwise.
// /!\ do not call too often
ldStruct laserDetect(bufStruct *bs);

//function to call periodically, ensures acquisition and tracking of our laser beam
int periodicLaser(bufStruct *bs,plStruct *pRet);

//float laser2dist(unsigned long delta);

#endif /* LIB_INT_LASER_H_ */
