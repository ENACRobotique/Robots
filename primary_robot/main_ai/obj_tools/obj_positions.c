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
#include "obj_positions.h"
#include "obj_time_tools.h"

typedef struct sPosListEl sPosListEl;
struct sPosListEl {
    sGenericPos pos;

    struct sPosListEl *next;
};

// TODO call handlers on position update
static sPosListEl *lastPGPositions[NUM_E_ELEMENT] = {NULL};
static sPosListEl *lastPRPositions = NULL;

static sPosListEl *newEl(){
    return (sPosListEl *)calloc(1, sizeof(sPosListEl));
}

static sPosListEl *newElData(sGenericPos *pos){
    sPosListEl *el;

    assert(pos);

    el = (sPosListEl *)malloc(sizeof(sPosListEl));

    memcpy(&el->pos, pos, sizeof(el->pos));
    el->next = NULL;

    return el;
}

static int comp(sGenericPos *e1, sGenericPos *e2){
    return time_diff(e1->date, e2->date);
}

// adds element sorted by date
static sPosListEl *addSorted(sPosListEl *head, sPosListEl *el){
    sPosListEl *curr, *prev;

    assert(el);

    for(prev = NULL, curr = head; curr && (!prev || comp(&prev->pos, &curr->pos)>0); prev = curr, curr = curr->next);

    if(curr && prev){
        prev->next = el;
        el->next = curr;

        return head;
    }
    else if(curr){
        el->next = curr;

        return el;
    }

    return head;
}

static sPosListEl *addHead(sPosListEl *head, sPosListEl *el){
    assert(el);

    el->next = head;
    return el;
}

// get last position of an element in the playground frame
sGenericPos *getLastPGPosition(eElement el){
    assert(el >= 0 && el < NUM_E_ELEMENT);

    return lastPGPositions[el] ? &lastPGPositions[el]->pos : NULL;
}

void fromPRPG2PG(sGenericPos *srcPR, sGenericPos *srcPG, sGenericPos *dstPG){
    float theta;

    assert(srcPR && srcPG && dstPG);
    assert(srcPR->frame == FRAME_PRIMARY && srcPG->frame == FRAME_PLAYGROUND);

    theta = srcPG->theta - M_PI/2.;

    dstPG->date = srcPR->date;
    dstPG->id = srcPR->id;
    dstPG->x = cos(theta)*srcPR->x - sin(theta)*srcPR->y + srcPG->x;
    dstPG->y = sin(theta)*srcPR->x + cos(theta)*srcPR->y + srcPG->y;
    dstPG->theta = theta + srcPR->theta;
    dstPG->frame = FRAME_PLAYGROUND;

    // TODO compute full uncertainty
    dstPG->u_theta = srcPR->u_theta + srcPG->u_theta;
}

void positions_maintenance(){
    sPosListEl *tmp;
    // TODO delete old items of the lists (for ex. older than 5seconds)
    // and retry asking primary position after timeout (only if this is the last PR pos of the element)

    for(tmp = lastPRPositions; tmp; tmp = tmp->next){
        if(tmp->pos.date){
// TODO compare but need synchronized time (as well as in obj_tim_tools.c, I'm going to do it... and come back here later)
        }
    }
}

int received_new_generic_pos(sGenericPos *pos){
    sMsg outMsg;
    int ret = 0;
    sPosListEl *tmp = NULL;

    assert(pos);

    switch(pos->frame){
    case FRAME_PRIMARY:
        lastPRPositions = addHead(lastPRPositions, newElData(pos));

        if(pos->id != ELT_PRIMARY){
            // search for a matching primary robot position
            for(tmp = lastPGPositions[ELT_PRIMARY]; tmp; tmp = tmp->next){
                if(fabs(time_diff(tmp->pos.date, pos->date)) < SAME_DATE_THRESHOLD){
                    sPosListEl *newPGEl = newEl();

                    fromPRPG2PG(pos, &tmp->pos, &newPGEl->pos);

                    lastPGPositions[pos->id] = addSorted(lastPGPositions[pos->id], newPGEl);
                    break;
                }
            }

            // if couldn't find a matching primary robot position
            if(!tmp){
                // ask matching position to the propulsion
                outMsg.header.destAddr = role_get_addr(ROLE_PROPULSION);
                outMsg.header.type = E_POS_QUERY;
                outMsg.header.size = sizeof(outMsg.payload.posQuery);

                outMsg.payload.posQuery.date = pos->date;
                outMsg.payload.posQuery.id = ELT_PRIMARY;

                ret = bn_send(&outMsg);
                if(ret < 0){
                    return ret;
                }
            }
        }
        break;
    case FRAME_PLAYGROUND:
        lastPGPositions[pos->id] = addSorted(lastPGPositions[pos->id], newElData(pos));

        if(pos->id == ELT_PRIMARY){
            // search for a matching item with this received robot position
            for(tmp = lastPRPositions; tmp; tmp = tmp->next){
                if(fabs(time_diff(tmp->pos.date, pos->date)) < SAME_DATE_THRESHOLD){
                    sPosListEl *newPGEl = newEl();

                    fromPRPG2PG(&tmp->pos, pos, &newPGEl->pos);

                    lastPGPositions[tmp->pos.id] = addSorted(lastPGPositions[tmp->pos.id], newPGEl);
                }
            }
        }
        break;
    default:
        return -1;
    }

    positions_maintenance();

    return ret;
}
