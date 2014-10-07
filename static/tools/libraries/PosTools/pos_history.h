/*
 * pos_history.h
 *
 *  Created on: 8 juin 2014
 *      Author: ludo6431
 */

#ifndef LIB_POSTOOLS_POS_HISTORY_H_
#define LIB_POSTOOLS_POS_HISTORY_H_

#include <messages-statuses.h>
#include <stdint.h>
#include <string.h>
#include <time_tools.h>

// keep history of about 500ms (at a rate of one position per 20ms)
// +1 because we have n-1 intervals in between n points
// +1 because we sacrifice one element to avoid managing the number of elements
#define NB_PREVIOUS_POSITIONS (500/20 + 1 + 1)

typedef struct {
    sGenericStatus s;
    sDate d;
} sPHPos;
extern sPHPos ph_cbuf[NB_PREVIOUS_POSITIONS];
extern uint8_t ph_newest;

static inline sPHPos *ph_get_new_slot_pointer(){
    ph_cbuf[ph_newest].d = tD_new(); // i.e.: new == invalid

    return &ph_cbuf[ph_newest];
}

static inline void ph_incr_new_slot_pointer(){
    ph_cbuf[ph_newest].d = tD_conv_Lo(ph_cbuf[ph_newest].d);

    ph_newest = (ph_newest + 1)%NB_PREVIOUS_POSITIONS;
}

static inline void ph_enqueue(sGenericStatus *s, sDate t){
    sPHPos *new = ph_get_new_slot_pointer();

    memcpy(&new->s, s, sizeof(*new));
    new->d = t;

    ph_incr_new_slot_pointer();
}

int ph_get_pos(sGenericStatus *s, sDate date);

#endif /* LIB_POSTOOLS_POS_HISTORY_H_ */
