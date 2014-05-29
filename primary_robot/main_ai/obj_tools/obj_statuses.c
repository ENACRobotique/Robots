/*
 * obj_position.c
 *
 *  Created on: 18 avr. 2014
 *      Author: ludo6431
 */

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <messages.h>
#include <roles.h>
#include <malloc.h>
#include <shared/botNet_core.h>
#include <shared/message_header.h>
#include "obj_time_tools.h"

#include "obj_statuses.h"

typedef struct sStatusListEl sStatusListEl;
struct sStatusListEl {
    struct sStatusListEl *next;

    sGenericStatus status;
};

typedef struct{
    // config
    sStatusHandlingConfig cfg;

    // instance's variables
    sStatusListEl *lastPGstatuses;
    sStatusListEl *lastStatuses;
} sStatusListInstance;

sStatusListInstance elts[NUM_E_ELEMENT];
static sStatusListEl *lastPRStatuses = NULL;

static sStatusListEl *newEl(){
    return (sStatusListEl *)calloc(1, sizeof(sStatusListEl));
}

static sStatusListEl *newElData(sGenericStatus *status){
    sStatusListEl *el;

    assert(status);

    el = (sStatusListEl *)malloc(sizeof(sStatusListEl));

    memcpy(&el->status, status, sizeof(el->status));
    el->next = NULL;

    return el;
}

static int comp(sGenericStatus *e1, sGenericStatus *e2){
    return time_diff(e1->date, e2->date);
}

// adds element sorted by date
static sStatusListEl *addSorted(sStatusListEl *head, sStatusListEl *el){
    sStatusListEl *curr, *prev;

    assert(el);

    for(prev = NULL, curr = head; curr && (!prev || comp(&prev->status, &curr->status)>0); prev = curr, curr = curr->next);

    if(prev){ // add after an element
        prev->next = el;
        el->next = curr;

        return head;
    }
    // add on first position
    el->next = curr;
    return el;
}

static sStatusListEl *addHead(sStatusListEl *head, sStatusListEl *el){
    assert(el);

    el->next = head;
    return el;
}

// get last position of an element in the playground frame
sGenericStatus *getLastStatus(eElement el){
    assert(el >= 0 && el < NUM_E_ELEMENT);

    return elts[el].lastStatuses ? &elts[el].lastStatuses->status : NULL;
}

// get last position of an element in the playground frame
sGenericStatus *getLastPGStatus(eElement el){
    assert(el >= 0 && el < NUM_E_ELEMENT);

    return elts[el].lastPGstatuses ? &elts[el].lastPGstatuses->status : NULL;
}

// TODO sGenericPos *popLastPGPosition(eElement el)

void fromPRPG2PG(s2DPosAtt *srcPAPR, s2DPAUncert *srcUPR, s2DPosAtt *srcPAPG, s2DPAUncert *srcUPG, s2DPosAtt *dstPAPG, s2DPAUncert *dstUPG){
    float theta;

    assert(srcPAPR && srcPAPG);
    assert(srcPAPR->frame == FRAME_PRIMARY && srcPAPG->frame == FRAME_PLAYGROUND);

    // create position if asked to
    if(dstPAPG){
        theta = srcPAPG->theta - M_PI/2.;
        dstPAPG->x = cos(theta)*srcPAPR->x - sin(theta)*srcPAPR->y + srcPAPG->x;
        dstPAPG->y = sin(theta)*srcPAPR->x + cos(theta)*srcPAPR->y + srcPAPG->y;
        dstPAPG->theta = theta + srcPAPR->theta;
        dstPAPG->frame = FRAME_PLAYGROUND;
    }

    // create uncertainty if asked to
    if(srcUPR && srcUPG && dstUPG){
        dstUPG->theta = srcUPR->theta + srcUPG->theta;
        // TODO compute full uncertainty
    }
}

void statuses_maintenance(){
//    sPosListEl *tmp;
    // TODO delete old items of the lists (for ex. older than 5seconds)
    // and retry asking primary position after timeout (only if this is the last PR pos of the element)

//    for(tmp = lastPRPositions; tmp; tmp = tmp->next){
//        if(tmp->pos.date){
// TODO compare but need synchronized time (as well as in obj_tim_tools.c, I'm going to do it... and come back here later)
//        }
//    }
}

sStatusHandlingConfig *getConfig(eElement el){
    return &elts[el].cfg;
}

void setConfig(eElement el, sStatusHandlingConfig *cfg){
    if(cfg){
        memcpy(&elts[el].cfg, cfg, sizeof(elts[el].cfg));
    }
    else{
        memset(&elts[el].cfg, 0, sizeof(elts[el].cfg));
    }
}

statusHandler setPGHandler(eElement el, statusHandler h){
    statusHandler prevH;

    assert(el >=0 && el < NUM_E_ELEMENT);

    prevH = elts[el].cfg.handlerPG;
    elts[el].cfg.handlerPG = h;

    return prevH;
}

int received_new_status(sGenericStatus *status){
//    sMsg outMsg;
    int ret = 0;
    sStatusListEl *tmp = NULL;

    assert(status);
    assert(status->id >= 0 && status->id < NUM_E_ELEMENT);

    if(elts[status->id].cfg.has_position){
        switch(status->pos.frame){
        case FRAME_PRIMARY:
            lastPRStatuses = addHead(lastPRStatuses, newElData(status));

            if(status->id != ELT_PRIMARY){
    #if 0
                // search for a matching primary robot position
                for(tmp = lastPGStatuses[ELT_PRIMARY]; tmp; tmp = tmp->next){
                    if(fabs(time_diff(tmp->status.date, status->date)) < SAME_DATE_THRESHOLD){
                        sStatusListEl *newPGEl = newEl();

                        fromPRPG2PG(status, &tmp->status, &newPGEl->status);

                        lastPGStatuses[status->id] = addSorted(lastPGStatuses[status->id], newPGEl);
                        break;
                    }
                }
                // if couldn't find a matching primary robot position
                if(!tmp){
                    // ask matching position to the propulsion
                    outMsg.header.destAddr = role_get_addr(ROLE_PROPULSION);
                    outMsg.header.type = E_POS_QUERY;
                    outMsg.header.size = sizeof(outMsg.payload.posQuery);

                    outMsg.payload.posQuery.date = status->date;
                    outMsg.payload.posQuery.id = ELT_PRIMARY;

                    ret = bn_send(&outMsg);
                    if(ret < 0){
                        return ret;
                    }
                }
    #else // XXX approximation: uses last known position of the primary robot instead of the time-matching one
                sStatusListEl *newPGEl = newEl();
                tmp = elts[ELT_PRIMARY].lastPGstatuses;

//                printf("lastPrimPGstatus=%p\n", tmp);

                if(newPGEl && tmp){
                    fromPRPG2PG(&status->pos, &status->pos_u, &tmp->status.pos, &tmp->status.pos_u, &newPGEl->status.pos, &newPGEl->status.pos_u);
                    newPGEl->status.date = status->date;
                    newPGEl->status.id = status->id;
                    elts[status->id].lastPGstatuses = addSorted(elts[status->id].lastPGstatuses, newPGEl);

//                    printf("lastPRstatus={%.2f,%.2f}\n", status->pos.x, status->pos.y);
//                    printf("lastPrimPGstatus={%.2f,%.2f}\n", tmp->status.pos.x, tmp->status.pos.y);
//                    printf("lastNew(%i)PGstatus={%.2f,%.2f}\n", status->id, newPGEl->status.pos.x, newPGEl->status.pos.y);

                    if(elts[status->id].cfg.handlerPG){
                        elts[status->id].cfg.handlerPG(&newPGEl->status);
                    }
                }
    #endif
            }
            break;
        case FRAME_PLAYGROUND:
            elts[status->id].lastPGstatuses = addSorted(elts[status->id].lastPGstatuses, newElData(status));

            if(elts[status->id].cfg.handlerPG){
                elts[status->id].cfg.handlerPG(status);
            }

    #if 0
            if(status->id == ELT_PRIMARY){
                // search for a matching item with this received robot position
                for(tmp = lastPRStatuses; tmp; tmp = tmp->next){
                    if(fabs(time_diff(tmp->status.date, status->date)) < SAME_DATE_THRESHOLD){
                        sStatusListEl *newPGEl = newEl();

                        fromPRPG2PG(&tmp->status, status, &newPGEl->status);

                        lastPGStatuses[tmp->status.id] = addSorted(lastPGStatuses[tmp->status.id], newPGEl);
                    }
                }
            }
#endif
            break;
        default:
            return -1;
        }
    }

    statuses_maintenance();

    return ret;
}
