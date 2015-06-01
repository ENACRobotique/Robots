/*
 ============================================================================
 Name        : test_uncertainty.c
 Author      : Ludovic Lacoste
 ============================================================================
 */

#include <math.h>
#include <messages-statuses.h>
#include <stdio.h>
#include <stdlib.h>
#include <time_tools.h>

#include <pos_uncertainty.h>

void dump_gstatus(sGenericPosStatus *gs, char *prefix){
    s2DPUncert_icovar o;
    sGenericPosStatus oo = { 0 };
    gstatus2icovar(gs, &o);
    icovar2gstatus(&o, &oo);

    printf("%s  a:% -15.5g\n", prefix, o.a);
    printf("%s  b:% -15.5g\n", prefix, o.b);
    printf("%s  c:% -15.5g\n", prefix, o.c);
    printf("%s  d:% -15.5g\n", prefix, o.d);

    printf("%s     BEFORE        AFTER\n", prefix);
    printf("%spos:\n", prefix);
    printf("%s  x:% -10.5fcm  % -10.5f\n", prefix, gs->pos.x, oo.pos.x);
    printf("%s  y:% -10.5fcm  % -10.5f\n", prefix, gs->pos.y, oo.pos.y);
    printf("%s  t:% -10.5f°   % -10.5f\n", prefix, gs->pos.theta * 180. / M_PI, oo.pos.theta * 180. / M_PI);
    printf("%spos_u:\n", prefix);
    printf("%s  av:% -10.5fcm² % -10.5f\n", prefix, gs->pos_u.a_var, oo.pos_u.a_var);
    printf("%s  bv:% -10.5fcm² % -10.5f\n", prefix, gs->pos_u.b_var, oo.pos_u.b_var);
    printf("%s  an:% -10.5f°   % -10.5f\n", prefix, gs->pos_u.a_angle * 180. / M_PI, oo.pos_u.a_angle * 180. / M_PI);
    printf("%s  tv:% -10.5f°²  % -10.5f\n", prefix, gs->pos_u.theta_var * powf(180. / M_PI, 2), oo.pos_u.theta_var * powf(180. / M_PI, 2));
}

float rand_uniform(float min, float max){
    return ((double) rand() / ((double)RAND_MAX + 1)) * (max - min) + min;
}

int main(int argc, char *argv[]){
    sGenericPosStatus i1, i2, o;

    i1.id = ELT_PRIMARY;
    i1.date = TD_GET_LoUs(tD_newNow_Lo());
    i1.pos.frame = FRAME_PLAYGROUND;
    i1.pos.x = 100.;
    i1.pos.y = 100.;
    i1.pos.theta = 0. * M_PI / 180.;
    i1.pos_u.a_angle = -80. * M_PI / 180.;
    i1.pos_u.a_var = 100; // we are along a axis
    i1.pos_u.b_var = 0;
    i1.pos.theta = 0.;
    i1.pos_u.theta_var = 0.;

    printf("i1:\n");
    dump_gstatus(&i1, "  ");

    i2.id = ELT_PRIMARY;
    i2.date = i1.date;
    i2.pos.frame = FRAME_PLAYGROUND;
    i2.pos.x = 100.;
    i2.pos.y = 150.;
    i2.pos.theta = 0. * M_PI / 180.;
    i2.pos_u.a_angle = 0. * M_PI / 180.;
    i2.pos_u.a_var = 10;
    i2.pos_u.b_var = 10;
    i2.pos.theta = 0.;
    i2.pos_u.theta_var = 0.;

    printf("i2:\n");
    dump_gstatus(&i2, "  ");

    pos_uncertainty_mix(&i1, &i2, &o);

    printf("o:\n");
    dump_gstatus(&o, "  ");

    {
#define IS_NEAR(a, b, eps) (fabs((a) - (b)) <= fabs(a)*(eps) || (a) < 1e-25f)
#define EXPECT_NEAR_PERCENT(a, b, eps, err) do {                                                    \
        if(!IS_NEAR(a, b, eps)) {                                                                   \
            printf("!!!!i=%i  not near: %g%%, %s\n", i, fabs(((a) - (b))/((a)?(a):(b)))*100, err);  \
            printf("!!!! \""#a"\" evaluates to: %g\n", (a));                                        \
            printf("!!!! \""#b"\" evaluates to: %g\n", (b));                                        \
            dump_gstatus(&in, "!!!!  ");                                                            \
            exit(1);                                                                                \
        }                                                                                           \
    } while(0)
#define THRESHOLD (0.03f/100)

        s2DPUncert_icovar tmp;
        sGenericPosStatus in = { 0 }, out = { 0 };
        int i;
        for(i = 0; i < 10000; i++){
            in.id = ELT_PRIMARY;
            in.date = TD_GET_LoUs(tD_newNow_Lo());
            in.pos.frame = FRAME_PLAYGROUND;
            in.pos.x = rand_uniform(0, 300);
            in.pos.y = rand_uniform(0, 200);
            in.pos_u.a_angle = rand_uniform(-M_PI, M_PI);
            in.pos_u.a_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            in.pos_u.b_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            in.pos.theta = rand_uniform(-M_PI, M_PI);
            in.pos_u.theta_var = rand_uniform(MINVARIANCE_THETA, MAXVARIANCE_THETA);

            gstatus2icovar(&in, &tmp);
            icovar2gstatus(&tmp, &out);

            EXPECT_NEAR_PERCENT(in.pos.x, out.pos.x, THRESHOLD, "pos.x");
            EXPECT_NEAR_PERCENT(in.pos.y, out.pos.y, THRESHOLD, "pos.y");

            int j;
            s2DPosAtt pa;
            pa.frame = FRAME_PLAYGROUND;
            for(j = 0; j < 100; j++){
                pa.x = rand_uniform(0, 300);
                pa.y = rand_uniform(0, 200);
                pa.theta = rand_uniform(-M_PI, M_PI);

                s2DPAProbability ipap = pos_uncertainty_eval(&in, &pa);
                s2DPAProbability opap = pos_uncertainty_eval(&out, &pa);

                EXPECT_NEAR_PERCENT(ipap.xy_probability, opap.xy_probability, 0.02f, "xy_prob");
                EXPECT_NEAR_PERCENT(ipap.theta_probability, opap.theta_probability, THRESHOLD, "theta_prob");
            }
        }

        printf("OK!!!\n");
#undef IS_NEAR
#undef EXPECT_NEAR_PERCENT
#undef THRESHOLD
    }

    {
#define IS_NEAR(a, b, eps) (fabs((a) - (b)) <= (eps))
#define EXPECT_NEAR(aa, bb, eps, err) do {                                                      \
        if(!IS_NEAR(aa, bb, eps)) {                                                                     \
            printf("!!!!i=%i  not near: %g, %s\n", i, fabs((aa) - (bb)), err); \
            printf("!!!! \""#aa"\" evaluates to: %g\n", (aa));                                          \
            printf("!!!! \""#bb"\" evaluates to: %g\n", (bb));                                          \
            printf("!!!! av:%.4f ; bv:%4f\n", x_var, y_var);                                            \
            exit(1);                                                                                    \
        }                                                                                               \
    } while(0)
#define THRESHOLD_VAR (1)
#define THRESHOLD_XY (1)
#define THRESHOLD_THETA (1e-1)

        int i;
        for(i = 0; i < 10000; i++){
            s2DPUncert_covar in;
            sGenericPosStatus out;

            float x_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            float y_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            while(fabs(y_var - x_var) < 1e-2 * x_var){
                y_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            }
            {
                if(x_var < y_var){
                    float tmp = x_var;
                    x_var = y_var;
                    y_var = tmp;
                }
            }

            float x = rand_uniform(0, 300);
            float y = rand_uniform(0, 200);
            float x_angle = rand_uniform(-M_PI, M_PI);
            float cxa = cosf(x_angle), sxa = sinf(x_angle);

            in.x = x;
            in.y = y;
            in.a = cxa*cxa*x_var + sxa*sxa*y_var;
            in.b = cxa*sxa*(x_var - y_var);
            in.c = cxa*cxa*y_var + sxa*sxa*x_var;

            covar2gstatus(&in, &out);

//            printf("###############\n");
//            printf("  a = %.4f\n", in.a);
//            printf("  b = %.4f\n", in.b);
//            printf("  c = %.4f\n", in.c);
//            printf("  theta = %.4f\n", x_angle*180.f/M_PI);

            EXPECT_NEAR(x, out.pos.x, THRESHOLD_XY, "x");
            EXPECT_NEAR(y, out.pos.y, THRESHOLD_XY, "y");

            if(out.pos_u.a_var < out.pos_u.b_var){
                float tmp = out.pos_u.a_var;
                out.pos_u.a_var = out.pos_u.b_var;
                out.pos_u.b_var = tmp;
                out.pos_u.a_angle += M_PI/2.;
            }
            EXPECT_NEAR(x_var, out.pos_u.a_var, THRESHOLD_VAR, "a_var");
            EXPECT_NEAR(y_var, out.pos_u.b_var, THRESHOLD_VAR, "b_var");

            float da = x_angle - out.pos_u.a_angle;
            while(da > M_PI / 2.) da -= M_PI;
            while(da < -M_PI / 2.) da += M_PI;
            EXPECT_NEAR(da, 0., THRESHOLD_THETA, "a_angle");
        }

        printf("OK!!!\n");
#undef IS_NEAR
#undef EXPECT_NEAR
#undef THRESHOLD_THETA
#undef THRESHOLD_XY
#undef THRESHOLD_VAR
    }

    return 0;
}
