/*
 * pos_history.c
 *
 *  Created on: 8 juin 2014
 *      Author: ludo6431
 */

#include <math.h>
#include <pos_history.h>

// TODO be careful of date overflow
// TODO implement uncertainty mixing

struct {
    sGenericStatus s;
    uint8_t valid;
} ph_cbuf[NB_PREVIOUS_POSITIONS] = {{{0}, 0}};
uint8_t ph_newest = 0;

int ph_get_pos(sGenericStatus *s, uint32_t date){
    uint8_t before = ph_newest, after = (ph_newest + NB_PREVIOUS_POSITIONS - 1)%NB_PREVIOUS_POSITIONS, next;
    sGenericStatus *sb, *sa;
    uint32_t dtb, dta, dt;

    // find position just before the date provided
    next = (before + 1)%NB_PREVIOUS_POSITIONS;
    while((!ph_cbuf[before].valid || (int32_t)(date - ph_cbuf[next].s.date) > 0) && next != ph_newest){
        before = next;
        next = (before + 1)%NB_PREVIOUS_POSITIONS;
    }
    if(!ph_cbuf[before].valid || !(ph_cbuf[before].s.date < date)){
        return -1;
    }
    sb = &ph_cbuf[before].s;
    dtb = date - sb->date;

    // find position just after the date provided
    next = (after + NB_PREVIOUS_POSITIONS - 1)%NB_PREVIOUS_POSITIONS;
    while((!ph_cbuf[after].valid || ph_cbuf[next].s.date > date) && next != ph_newest){
        after = next;
        next = (after + NB_PREVIOUS_POSITIONS - 1)%NB_PREVIOUS_POSITIONS;
    }
    if(!ph_cbuf[after].valid || !(ph_cbuf[after].s.date > date)){
        return -1;
    }
    sa = &ph_cbuf[after].s;
    dta = sa->date - date;

    dt = sa->date - sb->date;

    // linear interpolation of positions and attitude to find new one
    s->date = date;
    s->id = sb->id;
    s->prop_status.pos.frame = FRAME_PLAYGROUND;
    s->prop_status.pos.x = (sa->prop_status.pos.x * (float)dtb + sb->prop_status.pos.x * (float)dta)/(float)dt;
    s->prop_status.pos.y = (sa->prop_status.pos.y * (float)dtb + sb->prop_status.pos.y * (float)dta)/(float)dt;
    {
        float tb = sb->prop_status.pos.theta;
        float ta = sa->prop_status.pos.theta;

        while(fabs(tb - (ta + M_TWOPI)) < fabs(tb - ta)){
            ta += M_TWOPI;
        }
        while(fabs(tb - (ta - M_TWOPI)) < fabs(tb - ta)){
            ta -= M_TWOPI;
        }
        s->prop_status.pos.theta = (ta * (float)dtb + tb * (float)dta)/(float)dt;
    }

    return 0;
}
