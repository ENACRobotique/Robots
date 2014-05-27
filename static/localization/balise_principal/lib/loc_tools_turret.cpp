/*
 * loc_tools_turret.cpp
 *
 *  Created on: 4 avr. 2014
 *      Author: quentin
 */

extern "C"{
#include "../../../../network_config/roles.h"
}
#include "loc_tools_turret.h"
#include "lib_domitille.h"
#include "../../../global_errors.h"
#include "../../../communication/network_tools/bn_debug.h"
#include "../../../communication/botNet/shared/botNet_core.h"
#include "math.h"
#include "Arduino.h"
#include "params.h"


/* Converts a time value to a angle in radian, based on the last few recorded turns of the turret
 * Argument :
 *  time : time (synchronized) at which the beacon has seen the laser passing
 * Return value :
 *  0 if correct and *ret is modified
 *  <0 is error.
 *
 */
int time2rad(uint32_t time, float *ret){
    int tempIndex=0;
    int tempIndexI=-1;
    int err=0;
    int tries=0;
    uint32_t tempPeriod=0,tempDate=0;
#ifdef DEBUG_LOC
    uint32_t earliest=0,oldest=0;
#endif
    // to be sure that we will not be impaired by an update in interruption, we detect them
    while (tempIndexI!=TR_iNext && err!=-ERR_NOT_FOUND){       // Deadlock if we take more than 33ms to perform this block
        tempIndex=TR_iNext;
        tempIndexI=tempIndex;
        err=0;
        tries=0;


        // sweep TR_infoBuf to find the appropriate time interval
        while ( tries!=TR_INFO_BUFFER_SIZE && (TR_InfoBuf[tempIndex].date+TR_InfoBuf[tempIndex].period)<time){ //fixme overflow error (test instead  : "(time-TR_...date)<TR_...period" ?)
            tempIndex=(tempIndex+1)%TR_INFO_BUFFER_SIZE;
            tries++;
        }
        if (tries==0){ // if too late
            err=-ERR_NOT_FOUND;
        }
        else if (tries==TR_INFO_BUFFER_SIZE) { // if too early
#ifdef DEBUG_LOC
            earliest=TR_InfoBuf[TR_iNext].date;
            oldest=(TR_InfoBuf[tempIndex].date+TR_InfoBuf[tempIndex].period);
#endif
            err=-ERR_TRY_AGAIN;
        }
        else {
            tempPeriod=TR_InfoBuf[tempIndex].period;
            tempDate=TR_InfoBuf[tempIndex].date;
        }
    }
    if (err) {
#ifdef DEBUG_LOC
        bn_printfDbg((char*)"err %d early %lu old %lu time %lu\n",err,earliest,oldest,time);
#endif
        return err;
    }

    // Compute the angle
    else {
        *ret=((time-tempDate)*2*M_PI/tempPeriod);
#ifdef DEBUG_LOC
        bn_printfDbg((char*)"%lu,%lu,%lu,%lu,%d°\n",micros(),TR_InfoBuf[tempIndex].date,TR_InfoBuf[tempIndex].period,time,(int)(*ret*180./M_PI));
#endif
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

    // robot's geometry correction
    angle-=ANGLE_ZERO;
    if (angle<0) angle+=2*M_PI;

    sMsg msg={{0}};
    msg.header.size=sizeof(sGenericStatus);
    msg.header.type=E_GENERIC_STATUS;
    msg.header.destAddr=role_get_addr(ROLE_IA);

    msg.payload.genericStatus.date=pLoad->date;        // todo : synchronize this with ia
    msg.payload.genericStatus.id=(origin==ADDRX_MOBILE_1?ELT_ADV_PRIMARY:ELT_ADV_SECONDARY);

    msg.payload.genericStatus.adv_status.pos.x=(float)(pLoad->value)/10.*sin(angle);
    msg.payload.genericStatus.adv_status.pos.y=(float)(pLoad->value)/10.*cos(angle);
    msg.payload.genericStatus.adv_status.pos.theta=0;
    msg.payload.genericStatus.adv_status.pos.frame=msg.payload.genericStatus.adv_status.pos.FRAME_PRIMARY;

    msg.payload.genericStatus.adv_status.pos_u.a_angle=-1;
    msg.payload.genericStatus.adv_status.pos_u.a_std=-1;
    msg.payload.genericStatus.adv_status.pos_u.b_std=-1;
    msg.payload.genericStatus.adv_status.pos_u.theta=-1;


    //bn_send(&msg);

    bn_printfDbg((char*)"%hx : (%lu,%d) (%d,%d)", origin, pLoad->value, (int)(angle*180./M_PI),(int)msg.payload.genericStatus.adv_status.pos.x,(int)msg.payload.genericStatus.adv_status.pos.y);

    return 0;

}
