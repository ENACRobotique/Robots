/*
 * messages-position.h
 *
 *  Created on: 6 oct. 2014
 *      Author: yoyo
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_VIDEOS_H_
#define LIB_NETWORK_CONFIG_MESSAGES_VIDEOS_H_

#include <stdint.h>
#include "tools/AbsPos2D.h"
#include "messages-objects.h"

typedef enum eVidTypeProc{
    PROC_POS,
    PROC_OBJ
} eVidTypeProc;

typedef struct sVidReq{
    eVidTypeProc type;
    AbsPos2D<double> pose;
    AbsPos2D<double> pose_u;
} sVidReq; // does not excess 56 bytes

typedef struct sVidPosResp{
    bool sucess;
    AbsPos2D<double> pose;
    AbsPos2D<double> pose_u;
} sVidPosResp; // does not excess 56 bytes

typedef struct sVidObjResp{
    uint8_t nbMaxObj;
    uint8_t nbCurObj;
    sObject obj;
} sVidObjResp; // does not excess 56 bytes



#endif /* LIB_NETWORK_CONFIG_MESSAGES_VIDEOS_H_ */
