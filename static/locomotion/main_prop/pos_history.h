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

// keep history of about 500ms (at a rate of one position per 20ms)
#define NB_PREVIOUS_POSITIONS (500/20)

extern sGenericStatus ph_cbuf[];
extern uint8_t ph_newest;

static inline sGenericStatus *ph_get_new_slot_pointer(){
    ph_cbuf[ph_newest].date = 0;

    return &ph_cbuf[ph_newest];
}

static inline void ph_incr_new_slot_pointer(){
    ph_newest = (ph_newest + 1)%NB_PREVIOUS_POSITIONS;
}

static inline void ph_enqueue(sGenericStatus *s){
    sGenericStatus *new = ph_get_new_slot_pointer();

    memcpy(new, s, sizeof(*new));

    ph_incr_new_slot_pointer();
}

int ph_get_pos(sGenericStatus *s, uint32_t date);

#endif /* POS_HISTORY_H_ */
