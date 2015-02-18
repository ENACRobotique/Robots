/*
 * obj_position.h
 *
 *  Created on: 18 avr. 2014
 *      Author: ludo6431
 */

#ifndef OBJ_POSITION_H_
#define OBJ_POSITION_H_

extern "C"{
#include "messages-statuses.h"
}

// in µs
#define SAME_DATE_THRESHOLD (1000)
#define KEEP_OLD_THRESHOLD (5*1000*1000)
#define PROP_ANSWER_TIMEOUT (2*1000*1000)

typedef void (*statusHandler)(sGenericStatus *);
typedef struct {
        uint8_t has_position;
        statusHandler handlerPG;

        // TODO add list maintenance parameters
} sStatusHandlingConfig;

sGenericStatus *getLastStatus(eElement el);
sGenericStatus *getLastPGStatus(eElement el);
void fromPRPG2PG(s2DPosAtt *srcPAPR, s2DPAUncert *srcUPR, s2DPosAtt *srcPAPG, s2DPAUncert *srcUPG, s2DPosAtt *dstPAPG, s2DPAUncert *dstUPG);
void statuses_maintenance();
sStatusHandlingConfig *getConfig(eElement el);
void setConfig(eElement el, sStatusHandlingConfig *cfg);
statusHandler setPGHandler(eElement el, statusHandler h);
int received_new_status(sGenericStatus *status);

#endif /* OBJ_POSITION_H_ */
