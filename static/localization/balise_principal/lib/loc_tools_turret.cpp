/*
 * loc_tools_turret.cpp
 *
 *  Created on: 4 avr. 2014
 *      Author: quentin
 */

#include "loc_tools_turret.h"
#include "lib_domitille.h"
#include "../../../global_errors.h"
#include "../../../communication/botNet/shared/bn_debug.h"
#include "math.h"


/* Converts a time value to a angle in radian, based on the last few recorded turns of the turret
 * Argument :
 *  time : time (synchronized) at which the beacon has seen the laser passing
 * Return value :
 *  0 if correct and *ret is modified
 *  <0 is error.
 *
 */
int time2rad(uint32_t time, float *ret){
    int tempIndex;
    int tempIndexI=-1;
    int err=0;
    int tries=0;
    uint32_t tempPeriod=0,tempDate=0;

    // to be sure that we will not be impaired by an update in interruption, we detect them
    while (tempIndexI!=TR_iNext && err!=-ERR_NOT_FOUND){       // Deadlock if we take more than 33ms to perform this block
        tempIndex=TR_iNext;
        tempIndexI=tempIndex;
        err=0;
        tries=0;


        // sweep TR_infoBuf to find the appropriate time interval
        while ( tries!=(TR_INFO_BUFFER_SIZE-1) && (TR_InfoBuf[tempIndex].date+TR_InfoBuf[tempIndex].period)<time){
            tempIndex=(tempIndex+1)%TR_INFO_BUFFER_SIZE;
            tries++;
        }
        if (tries==0){ // if too late
            err=-ERR_NOT_FOUND;
        }
        else if (tries==(TR_INFO_BUFFER_SIZE-1)) { // if too early
            err=-ERR_TRY_AGAIN;
        }
        else {
            tempPeriod=TR_InfoBuf[tempIndex].period;
            tempDate=TR_InfoBuf[tempIndex].date;
        }
    }
    if (err) return err;

    // Compute the angle
    else {
        *ret=((time-tempDate)*2*M_PI/tempPeriod);
        return 0;
    }

}


/* Makes the required computations and sends the message to IA with the measured position
 * Argument :
 *  pLoad : pointer to the data send by remote beacon
 * Return value :
 *  0 : everything went fine, *Pload can be cleared, the message to IA has been send
 *  <0 : something went wrong. Test return value to know if retries would be a good thing.
 */
int handleMeasurePayload(sMobileReportPayload *pLoad, bn_Address origin){
    float angle=0;
    int err=0;
    if ((err=time2rad(pLoad->date,&angle))){
        return err;
    }

    // fixme send to actual IA
    bn_printfDbg((char*)"%hx is at %lu mm %d  ", origin, pLoad->value, (int)angle*1000);
    return 0;

}
