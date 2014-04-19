/*
 * obj_position.c
 *
 *  Created on: 18 avr. 2014
 *      Author: ludo6431
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <messages.h>
#include <roles.h>
#include <malloc.h>
#include <shared/botNet_core.h>
#include <shared/message_header.h>
#include "obj_position.h"
#include "obj_time_tools.h"

typedef struct sPosListEl sPosListEl;
struct sPosListEl {
    sGenericPos pos;

    struct sPosListEl *next;
};

// TODO sort those lists (last element is first)
static sPosListEl *lastPGPositions[NUM_E_ELEMENT] = {NULL};
static sPosListEl *lastMNPositions = NULL;

static sPosListEl *newEl(){
    return (sPosListEl *)calloc(1, sizeof(sPosListEl));
}

static sPosListEl *newElData(sGenericPos *pos){
    sPosListEl *el;

    el = (sPosListEl *)malloc(sizeof(sPosListEl));

    memcpy(&el->pos, pos, sizeof(el->pos));
    el->next = NULL;

    return el;
}

// adds element sorted by date
static sPosListEl *addSorted(sPosListEl *head, sPosListEl *el){
    return NULL; // TODO
}

// get last position of an elelment in the playground frame
sGenericPos *getLastPGPosition(eElement el){
    return lastPGPositions[el] ? &lastPGPositions[el]->pos : NULL;
}

int received_new_generic_pos(sGenericPos *pos){
    sMsg outMsg;
    int ret = 0;
    sPosListEl *tmp = NULL;

    switch(pos->frame){
    case FRAME_PRIMARY:
        lastMNPositions = addSorted(lastMNPositions, newElData(pos));

        if(pos->id != ELT_PRIMARY){
            // search for a matching primary robot position
            for(tmp = lastPGPositions[ELT_PRIMARY]; tmp; tmp = tmp->next){
                if(fabs(time_diff(tmp->pos.date, pos->date)) < SAME_DATE_THRESHOLD){
                    sPosListEl *newPGEl = newEl();

                    newPGEl->pos.date = pos->date;
                    newPGEl->pos.id = pos->id;
                    // TODO fill newPGEl with new position in playground from tmp->pos (FRAME_PRIMARY) and pos (FRAME_PLAYGROUND)

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
            for(tmp = lastMNPositions; tmp; tmp = tmp->next){
                if(fabs(time_diff(tmp->pos.date, pos->date)) < SAME_DATE_THRESHOLD){
                    sPosListEl *newPGEl = newEl();

                    newPGEl->pos.date = tmp->pos.date;
                    newPGEl->pos.id = tmp->pos.id;
                    // TODO fill newPGEl with new position in playground from tmp->pos (FRAME_PRIMARY) and pos (FRAME_PLAYGROUND)

                    lastPGPositions[tmp->pos.id] = addSorted(lastPGPositions[tmp->pos.id], newPGEl);
                }
            }
        }
        break;
    default:
        return -1;
    }

    // TODO delete old items of the lists (for ex. older than 5seconds)

    return ret;
}
