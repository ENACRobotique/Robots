/*
 * pos_history.c
 *
 *  Created on: 8 juin 2014
 *      Author: ludo6431
 */

#include <math.h>
#include <messages-position.h>
#include <pos_history.h>

sPHPos ph_cbuf[NB_PREVIOUS_POSITIONS] = {{{0}, TD_CTOR()}};
uint8_t ph_newest = 0;
#define S(i) (ph_cbuf[(i)].s)
#define D(i) (ph_cbuf[(i)].d)
#define D_VALID(i) TD_VALID(D(i))
#define OF(i) ((NB_PREVIOUS_POSITIONS + (i))%NB_PREVIOUS_POSITIONS)

int ph_get_pos(sGenericPosStatus *s, sDate date){
    uint8_t before, after, next;
    sPHPos *phBefore, *phAfter;
    sPeriod periodBefore, periodAfter, periodFull;

    // find position just before the date provided
    before = ph_newest;
    next = OF(before + 1);
    while((!D_VALID(before) || !D_VALID(next) || TD_DIFF_Us(D(next), date) < 0) && next != ph_newest){
        before = next;
        next = OF(before + 1);
    }
    if(next == ph_newest){ // can't find position
        return -1;
    }
    phBefore = &ph_cbuf[before];
    periodBefore = tD_diff(date, phBefore->d);

    // find position just after the date provided
    after = OF(ph_newest - 1);
    next = OF(after - 1);
    while((!D_VALID(after) || !D_VALID(next) || TD_DIFF_Us(date, D(next)) < 0) && next != ph_newest){
        after = next;
        next = OF(after - 1);
    }
    if(next == ph_newest){
        return -1;
    }
    phAfter = &ph_cbuf[after];
    periodAfter = tD_diff(phAfter->d, date);

    periodFull = tD_diff(phAfter->d, phBefore->d);

    // linear interpolation of positions and attitude to find new one
    s->date = TD_GET_GlUs(date);
    s->id = phBefore->s.id;
    s->pos.frame = FRAME_PLAYGROUND;
    s->pos.x = (phAfter->s.pos.x * (float)TP_GET_Us(periodBefore) + phBefore->s.pos.x * (float)TP_GET_Us(periodAfter))/(float)TP_GET_Us(periodFull);
    s->pos.y = (phAfter->s.pos.y * (float)TP_GET_Us(periodBefore) + phBefore->s.pos.y * (float)TP_GET_Us(periodAfter))/(float)TP_GET_Us(periodFull);
    {
        float tb = phBefore->s.pos.theta;
        float ta = phAfter->s.pos.theta;

        while(tb - ta > M_PI){
            ta += 2.*M_PI;
        }
        while(tb - ta < -M_PI){
            ta -= 2.*M_PI;
        }
        s->pos.theta = (ta * (float)TP_GET_Us(periodBefore) + tb * (float)TP_GET_Us(periodAfter))/(float)TP_GET_Us(periodFull);
    }

    s->pos_u.a_var = (phAfter->s.pos_u.a_var * (float)TP_GET_Us(periodBefore) + phBefore->s.pos_u.a_var * (float)TP_GET_Us(periodAfter))/(float)TP_GET_Us(periodFull);
    s->pos_u.b_var = (phAfter->s.pos_u.b_var * (float)TP_GET_Us(periodBefore) + phBefore->s.pos_u.b_var * (float)TP_GET_Us(periodAfter))/(float)TP_GET_Us(periodFull);
    {
        float tb = phBefore->s.pos_u.a_angle;
        float ta = phAfter->s.pos_u.a_angle;

        while(tb - ta > M_PI){
            ta += 2.*M_PI;
        }
        while(tb - ta < -M_PI){
            ta -= 2.*M_PI;
        }
        s->pos_u.a_angle = (ta * (float)TP_GET_Us(periodBefore) + tb * (float)TP_GET_Us(periodAfter))/(float)TP_GET_Us(periodFull);
    }
    s->pos_u.theta_var = (phAfter->s.pos_u.theta_var * (float)TP_GET_Us(periodBefore) + phBefore->s.pos_u.theta_var * (float)TP_GET_Us(periodAfter))/(float)TP_GET_Us(periodFull);

    return 0;
}
