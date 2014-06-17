/*
 * pos_history.c
 *
 *  Created on: 8 juin 2014
 *      Author: ludo6431
 */

#include <math.h>
#include <pos_history.h>

// TODO implement uncertainty mixing

sPHPos ph_cbuf[NB_PREVIOUS_POSITIONS] = {{{0}, TIME_CTOR()}};
uint8_t ph_newest = 0;
#define S(i) (ph_cbuf[(i)].s)
#define T(i) (ph_cbuf[(i)].t)
#define T_VALID(i) TD_VALID(T(i))
#define OF(i) ((NB_PREVIOUS_POSITIONS + (i))%NB_PREVIOUS_POSITIONS)

int ph_get_pos(sGenericStatus *s, sDate date){
    uint8_t before, after, next;
    sPHPos *phBefore, *phAfter;
    sPeriod periodBefore, periodAfter, periodFull;

    // find position just before the date provided
    before = ph_newest;
    next = OF(before + 1);
    while((!T_VALID(before) || !T_VALID(next) || TD_GETDIFF(T(next), date) < 0) && next != ph_newest){
        before = next;
        next = OF(before + 1);
    }
    if(next == ph_newest){ // can't find position
        return -1;
    }
    phBefore = &ph_cbuf[before];
    periodBefore = tD_diff(date, phBefore->t); // in µs

    // find position just after the date provided
    after = OF(ph_newest - 1);
    next = OF(after - 1);
    while((!T_VALID(after) || !T_VALID(next) || TD_GETDIFF(date, T(next)) < 0) && next != ph_newest){
        after = next;
        next = OF(after - 1);
    }
    if(next == ph_newest){
        return -1;
    }
    phAfter = &ph_cbuf[after];
    periodAfter = tD_diff(phAfter->t, date); // in µs

    periodFull = tD_diff(phAfter->t, phBefore->t); // in µs

    // linear interpolation of positions and attitude to find new one
    s->date = TD_GET_GlUs(date);
    s->id = phBefore->s.id;
    s->prop_status.pos.frame = FRAME_PLAYGROUND;
    s->prop_status.pos.x = (phAfter->s.prop_status.pos.x * (float)TP_GET(periodBefore) + phBefore->s.prop_status.pos.x * (float)TP_GET(periodAfter))/(float)TP_GET(periodFull);
    s->prop_status.pos.y = (phAfter->s.prop_status.pos.y * (float)TP_GET(periodBefore) + phBefore->s.prop_status.pos.y * (float)TP_GET(periodAfter))/(float)TP_GET(periodFull);
    {
        float tb = phBefore->s.prop_status.pos.theta;
        float ta = phAfter->s.prop_status.pos.theta;

        while(fabs(tb - (ta + 2.*M_PI)) < fabs(tb - ta)){
            ta += 2.*M_PI;
        }
        while(fabs(tb - (ta - 2.*M_PI)) < fabs(tb - ta)){
            ta -= 2.*M_PI;
        }
        s->prop_status.pos.theta = (ta * (float)TP_GET(periodBefore) + tb * (float)TP_GET(periodAfter))/(float)TP_GET(periodFull);
    }

    return 0;
}
