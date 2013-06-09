
/****************************************************************
gestion des interruptions provenant des capteurs

pour arduino UNO
****************************************************************/

#include "Arduino.h"
#include "lib_int_laser.h"
#include "lib_time.h"
#include "tools.h"
#include "../src/params.h"

//globales
bufStruct buf0={{0},0,0,0,0,0,0,0};
bufStruct buf1={{0},0,0,0,0,0,0,1};



#define LASER_THICK_MIN 24    // in µs refined with measurement
#define LASER_THICK_MAX 600 // in µs refined with measurement

#define DEBOUNCETIME_INT_LASER 20  //measured
#define DEBUG_LASER

#define LAT_INIT 10000 //in µs TODO : refine



/*
laserIntInit :
  arguments : rien
  valeur de retour : rien
"attache" les interruptions associées aux couples de capteurs
*/
void laserIntInit(int irqnb) {
  pinMode(irqnb+2,INPUT);  // trick for the arduino UNO
  attachInterrupt(irqnb, !irqnb?laserIntHand0:laserIntHand1 ,CHANGE);
}
    
//do I really have to do a description ? Anyway, fuck this.
void laserIntDeinit(){
  detachInterrupt(0);
  detachInterrupt(1);
}

void laserIntHand0(){ //interrupt handler, puts the time in the rolling buffer
  //new! debouce, will hide any interruption happening less than DEBOUNCETIME µsec after the last registered interruption
	unsigned long time=micros();//mymicros();
	if ( time > (buf0.buf[buf0.index]+DEBOUNCETIME_INT_LASER) ){
        buf0.buf[buf0.index]=time;
        buf0.index++;
        buf0.index&=7;

	}
}

void laserIntHand1(){
  //new! debouce, will hide any interruption happening less than DEBOUNCETIME µsec after the last registered interruption
	unsigned long time=micros();//mymicros();
	if ( time > (buf1.buf[buf1.index]+DEBOUNCETIME_INT_LASER) ){
		buf1.buf[buf1.index]=time;
		buf1.index++;
		buf1.index&=7;
	}
}

//returns the delta-T in µs and the time at which it was measured
ldStruct laserDetect(bufStruct *bs){
	unsigned long prevCall, t = micros();
	unsigned long bufTemp[8], d1, d2;
	int i=8, ilast=0, nb;

	//noInterrupts();

	prevCall=bs->prevCall;
    do {
        i--;
        bufTemp[i]=bs->buf[i];
    } while(i);
    ilast=bs->index-1;//index of the last value written in the rolling buffer

	//interrupts();

	//looking for the index of first value updated since the last "interesting" call of laserDetect
	nb=0;
	i=ilast;
	while( nb<8 && long(bufTemp[i&7] - prevCall)>0 ) {
	i--;
	nb++;
	}

	bs->prevCall=t;

	if ( nb>=4 ){
	// we just got enough data, set the new update time
    d1 = bufTemp[ilast&7]-bufTemp[(ilast-2)&7];
    d2 = bufTemp[(ilast-1)&7]-bufTemp[(ilast-3)&7];
    unsigned long t1,t2;
    t1=bufTemp[ilast&7]-bufTemp[(ilast-1)&7];
    t2=bufTemp[(ilast-2)&7] - bufTemp[(ilast-3)&7];


    //if the detected patter has not the good shape
    if ( t1 < LASER_THICK_MIN || t1 > LASER_THICK_MAX || t2<LASER_THICK_MIN || t2>LASER_THICK_MAX ){
        ldStruct ret={0,0};
        return ret;
    }
    //
    if(d1<d2) {
        ldStruct ret={d1, bufTemp[(ilast-2)&7], min( t1, t2 )};
        return ret ;
    }
    else {
        ldStruct ret={d2, bufTemp[(ilast-3)&7], min( t1, t2 ) };
        return ret;
    }

	}
	else { //we don't have enough data,  just drop the current data <=> prevCall=t a few lines above
	}
    ldStruct ret={0,0};
    return ret;

}

/* Function co call periodically to poll if any new laser value
 * Argument :
 *  bs : pointer to the buffer structure to test
 *  pRet : pointer to the return structure. This latter is not modified if there is no new value
 * Return value : 1 if something new has been detected and written, 0 otherwise
 *
 */
int periodicLaser(bufStruct *bs,plStruct *pRet){
    unsigned long int time=micros();
    ldStruct measure;

    if ( time >= bs->nextTime){
        switch (bs->stage){
            case 0 : { //acquisition
                //laserdetect
                measure=laserDetect(bs);
                //if correct, goto tracking_clearing (stage 2), set the latency to LAT_INIT and the nextTime accordingly
                if (measure.deltaT!=0){
                    bs->stage=2;

                    bs->lat=LAT_INIT;
                    bs->prevTime=time;
                    bs->nextTime=time+laser_period- (bs->lat>>1);

                    pRet->deltaT=measure.deltaT;
                    pRet->date=measure.date;
                    pRet->thickness=measure.thickness;
                    pRet->sureness=laser_period;
                    pRet->precision=4; //in µs TODO

                    return 1;

                }
                //else, "delay" laserPeriod/2
                else {
                    //set the nextime and prevtime
                    bs->prevTime=time;
                    bs->nextTime=time+ ((3*laser_period)>>3); // NOT a period submultiple
                    return 0;
                }
                break;
            }
            case 1 :{ //tracking
                //laserdetect
                measure=laserDetect(bs);
                //if correct, decrease lat (unused), sets the delay to go to 2
                if (measure.deltaT!=0){
                    pRet->deltaT=measure.deltaT;
                    pRet->date=measure.date;
                    pRet->thickness=measure.thickness;
                    pRet->sureness=time-bs->prevTime+(bs->lat>>1); //sureness = difference between the expected time and the measured time
                    pRet->precision=4; //in µs TODO

                    bs->lat=LAT_INIT;    //MAX( bs->lat-LAT_DEINC,LAT_MIN);
                    bs->prevTime=time;
                    bs->nextTime=measure.date+laser_period-(bs->lat>>1);

                    bs->stage=2;

                    return 1;
                }
                //else, go to acquisition
                else {
                    bs->lat=LAT_INIT;  //bs->lat+LAT_INC;
                    bs->prevTime=time;
                    bs->nextTime=measure.date+laser_period-(bs->lat>>1);

                    bs->stage=0;

                    return 0;

                }
                break;
            }
            case 2 :{ //tracking : clear the buffer right before the beginning of the measurement time
                //"clear "the buffer
                bs->prevCall=time;

                //set the nextime and prevtime
                bs->prevTime=time;
                bs->nextTime=time+bs->lat;

                //return to stage 1
                bs->stage=1;

                return 0;
                break;
            }
            default : break;
        }
    }
return 0;
}


float laser2dist(unsigned long delta){
    return 0;//2.5/sin( (delta/laser_period-0.5*3.141593/180)/2);

}
