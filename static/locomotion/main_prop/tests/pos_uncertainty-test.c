/*
 * pos_uncertainty-test.c
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#include <math.h>
#include <messages-elements.h>
#include <messages-position.h>
#include <stdio.h>
#include <stdlib.h>
#include <time_tools.h>
#include "pos_uncertainty-test.h"
#include "test_tools.h"

#define POS_UNCERTAINTY_INTERNALS
#include <pos_uncertainty.h>

void dump_gstatus(sGenericStatus *gs, char *prefix) {
#ifdef HAS_POS_UNCERTAINTY_INTERNALS
    s2DPUncert_internal o;
    sGenericStatus oo = { 0 };
    gstatus2internal(gs, &o);
    internal2gstatus(&o, &oo);

    printf("%s  a:% -15.5g\n", prefix, o.a);
    printf("%s  b:% -15.5g\n", prefix, o.b);
    printf("%s  c:% -15.5g\n", prefix, o.c);

    printf("%s     BEFORE        AFTER\n", prefix);
    printf("%spos:\n", prefix);
    printf("%s  x:% -10.5fcm  % -10.5f\n", prefix, gs->pos.x, oo.pos.x);
    printf("%s  y:% -10.5fcm  % -10.5f\n", prefix, gs->pos.y, oo.pos.y);
    printf("%s  a:% -10.5f°   % -10.5f\n", prefix, gs->pos.theta * 180. / M_PI, oo.pos.theta * 180. / M_PI);
    printf("%spos_u:\n", prefix);
    printf("%s  av:% -9.5fcm² % -10.5f\n", prefix, gs->pos_u.a_var, oo.pos_u.a_var);
    printf("%s  bv:% -9.5fcm² % -10.5f\n", prefix, gs->pos_u.b_var, oo.pos_u.b_var);
    printf("%s  an:% -9.5f°   % -10.5f\n", prefix, gs->pos_u.a_angle * 180. / M_PI, oo.pos_u.a_angle * 180. / M_PI);
#else
    printf("%spos:\n", prefix);
    printf("%s  x:%.3fcm\n", prefix, gs->pos.x);
    printf("%s  y:%.3fcm\n", prefix, gs->pos.y);
    printf("%s  a:%.3f°\n", prefix, gs->pos.theta*180./M_PI);
    printf("%spos_u:\n", prefix);
    printf("%s  av:%.3fcm²\n", prefix, gs->pos_u.a_var);
    printf("%s  bv:%.3fcm²\n", prefix, gs->pos_u.b_var);
    printf("%s  an:%.3f°\n", prefix, gs->pos_u.a_angle*180./M_PI);
#endif
}

void testmain_pos_uncertainty() {
    sGenericStatus i1, i2, o;

    i1.id = ELT_PRIMARY;
    i1.date = TD_GET_LoUs(tD_newNow_Lo());
    i1.pos.frame = FRAME_PLAYGROUND;
    i1.pos.x = 100.;
    i1.pos.y = 100.;
    i1.pos_u.a_angle = -10. * M_PI / 180.;
    i1.pos_u.a_var = 4;
    i1.pos_u.b_var = 25;
    i1.pos.theta = 0.;
    i1.pos_u.theta = 0.;

    printf("i1:\n");
    dump_gstatus(&i1, "  ");

    i2.id = ELT_PRIMARY;
    i2.date = i1.date;
    i2.pos.frame = FRAME_PLAYGROUND;
    i2.pos.x = 101.34;
    i2.pos.y = 100.3;
    i2.pos_u.a_angle = 87.01 * M_PI / 180.;
    i2.pos_u.a_var = 0.05;
    i2.pos_u.b_var = 10;
    i2.pos.theta = 0.;
    i2.pos_u.theta = 0.;

    printf("i2:\n");
    dump_gstatus(&i2, "  ");

    pos_uncertainty_mix(&i1, &i2, &o);

    printf("o:\n");
    dump_gstatus(&o, "  ");

    {
#define EXPECT_NEAR_PERCENT(a, b, eps, err) do {if(!IS_NEAR_PERCENT(a, b, eps)) {\
    printf("!!!!i=%i  not near: %g%%, %s\n", i, fabs(((a) - (b))/(a))*100., err);\
    dump_gstatus(&in, "!!!!  ");\
    exit(1);\
}\
} while(0)

#define THRESHOLD (0.03/100.)

        s2DPUncert_internal tmp;
        sGenericStatus in = { 0 }, out = { 0 };
        int i;
        for (i = 0; i < 10000000; i++) {
            in.id = ELT_PRIMARY;
            in.date = TD_GET_LoUs(tD_newNow_Lo());
            in.pos.frame = FRAME_PLAYGROUND;
            in.pos.x = rand_uniform(0, 300);
            in.pos.y = rand_uniform(0, 200);
            in.pos_u.a_angle = rand_uniform(-M_PI, M_PI);
            in.pos_u.a_var = rand_uniform(0.01, 50);
            in.pos_u.b_var = rand_uniform(0.01, 50);
            in.pos.theta = 0.;
            in.pos_u.theta = 0.;

            gstatus2internal(&in, &tmp);
            internal2gstatus(&tmp, &out);

            EXPECT_NEAR_PERCENT(in.pos.x, out.pos.x, THRESHOLD, "pos.x");
            EXPECT_NEAR_PERCENT(in.pos.y, out.pos.y, THRESHOLD, "pos.y");

            if (fabs(in.pos_u.a_var - out.pos_u.a_var) < fabs(in.pos_u.a_var - out.pos_u.b_var)) {
                EXPECT_NEAR_PERCENT(in.pos_u.a_var, out.pos_u.a_var, THRESHOLD, "a_var");
                EXPECT_NEAR_PERCENT(in.pos_u.b_var, out.pos_u.b_var, THRESHOLD, "b_var");

                if(!IS_NEAR_PERCENT(in.pos_u.a_var, in.pos_u.b_var, 0.05/100)){
                    float out_pos_u_a_angle = out.pos_u.a_angle;
                    while ((out_pos_u_a_angle - in.pos_u.a_angle) > M_PI / 2) {
                        out_pos_u_a_angle -= M_PI;
                    }
                    while ((out_pos_u_a_angle - in.pos_u.a_angle) < -M_PI / 2) {
                        out_pos_u_a_angle += M_PI;
                    }

                    EXPECT_NEAR_PERCENT(in.pos_u.a_angle, out_pos_u_a_angle, THRESHOLD, "a_angle");
                }
            }
            else {
                EXPECT_NEAR_PERCENT(in.pos_u.a_var, out.pos_u.b_var, THRESHOLD, "!a_var");
                EXPECT_NEAR_PERCENT(in.pos_u.b_var, out.pos_u.a_var, THRESHOLD, "!b_var");

                if(!IS_NEAR_PERCENT(in.pos_u.a_var, in.pos_u.b_var, 0.05/100)){
                    float out_pos_u_a_angle = out.pos_u.a_angle - M_PI / 2;
                    while ((out_pos_u_a_angle - in.pos_u.a_angle) > M_PI / 2) {
                        out_pos_u_a_angle -= M_PI;
                    }
                    while ((out_pos_u_a_angle - in.pos_u.a_angle) < -M_PI / 2) {
                        out_pos_u_a_angle += M_PI;
                    }

                    EXPECT_NEAR_PERCENT(in.pos_u.a_angle, out_pos_u_a_angle, THRESHOLD, "!a_angle");
                }
            }
        }

        printf("OK!!!\n");
    }

}
