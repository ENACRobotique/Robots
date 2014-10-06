/*
 * pos_uncertainty-test.c
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#include <math.h>
#include <messages-elements.h>
#include <messages-position.h>
#include <messages-statuses.h>
#include <time_tools.h>
#include "pos_uncertainty-test.h"

void dump_gstatus(sGenericStatus *gs){
    printf("pos:\n");
    printf("\tx:%.3fcm\n\ty:%.3fcm\n\ta:%.3f°\n", gs->pos.x, gs->pos.y, gs->pos.theta*180./M_PI);
    printf("pos_u:\n");
    printf("\tav:%.3fcm²\n\tbv:%.3fcm²\n\tan:%.3f°\n", gs->pos_u.a_var, gs->pos_u.b_var, gs->pos_u.a_angle*180./M_PI);
}

void testmain_pos_uncertainty(){
    sGenericStatus i1, i2, o;

    i1.id = ELT_PRIMARY;
    i1.date = TD_GET_LoUs(tD_newNow_Lo());
    i1.pos.frame = FRAME_PLAYGROUND;
    i1.pos.x = 100.;
    i1.pos.y = 100.;
    i1.pos_u.a_angle = -10.*M_PI/180.;
    i1.pos_u.a_var = 4;
    i1.pos_u.b_var = 25;
    i1.pos.theta = 0.;
    i1.pos_u.theta = 0.;

    printf("i1:\n");
    dump_gstatus(&i1);

    i2.id = ELT_PRIMARY;
    i2.date = i1.date;
    i2.pos.frame = FRAME_PLAYGROUND;
    i2.pos.x = 102.;
    i2.pos.y = 103.;
    i2.pos_u.a_angle = 20.*M_PI/180.;
    i2.pos_u.a_var = 6;
    i2.pos_u.b_var = 8;
    i2.pos.theta = 0.;
    i2.pos_u.theta = 0.;

    printf("i2:\n");
    dump_gstatus(&i2);

    pos_uncertainty_mix(&i1, &i2, &o);

    printf("o:\n");
    dump_gstatus(&o);
}
