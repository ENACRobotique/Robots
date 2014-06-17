/*
 * pos_history.h
 *
 *  Created on: 8 juin 2014
 *      Author: ludo6431
 */

#ifndef POS_HISTORY_H_
#define POS_HISTORY_H_

#include <messages.h>
#include <stdint.h>
#include <string.h>
#include <time_tools.h>

// keep history of about 500ms (at a rate of one position per 20ms)
#define NB_PREVIOUS_POSITIONS (500/20)

typedef struct {
    sGenericStatus s;
    sDate t;
} sPHPos;
extern sPHPos ph_cbuf[NB_PREVIOUS_POSITIONS];
extern uint8_t ph_newest;

static inline sPHPos *ph_get_new_slot_pointer(){
    ph_cbuf[ph_newest].t = tD_new(); // i.e.: new == invalid

    return &ph_cbuf[ph_newest];
}

static inline void ph_incr_new_slot_pointer(){
    ph_cbuf[ph_newest].t = tD_conv_LoUs(ph_cbuf[ph_newest].t);

    ph_newest = (ph_newest + 1)%NB_PREVIOUS_POSITIONS;
}

static inline void ph_enqueue(sGenericStatus *s, sDate t){
    sPHPos *new = ph_get_new_slot_pointer();

    memcpy(&new->s, s, sizeof(*new));
    new->t = t;

    ph_incr_new_slot_pointer();
}

int ph_get_pos(sGenericStatus *s, sDate date);

#endif /* POS_HISTORY_H_ */
