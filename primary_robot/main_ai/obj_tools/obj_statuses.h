/*
 * obj_position.h
 *
 *  Created on: 18 avr. 2014
 *      Author: ludo6431
 */

#ifndef OBJ_POSITION_H_
#define OBJ_POSITION_H_

// in µs
#define SAME_DATE_THRESHOLD (1000)
#define KEEP_OLD_THRESHOLD (5*1000*1000)
#define PROP_ANSWER_TIMEOUT (2*1000*1000)

typedef void (*statusHandler)(sGenericStatus *);
typedef struct{
    uint8_t has_position;
    statusHandler handlerPG;

    // TODO add list maintenance parameters
} sStatusHandlingConfig;

sGenericStatus *getLastPGStatus(eElement el);

statusHandler setPGHandler(eElement id, statusHandler h);
int received_new_status(sGenericStatus *status);
void statuses_maintenance();
void setConfig(eElement el, sStatusHandlingConfig *cfg);

#endif /* OBJ_POSITION_H_ */
