/*
 ============================================================================
 Name        : test_uncertainty.c
 Author      : Ludovic Lacoste
 ============================================================================
 */

#include <math.h>
#include <string.h>
#include <messages-statuses.h>
#include <stdio.h>
#include <stdlib.h>
#include <time_tools.h>

#include <pos_uncertainty.h>

void dump_gstatus(sGenericPosStatus *gs, char *prefix){
#if 0
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
#else
    printf("%spos:\n", prefix);
    printf("%s  x:% -10.5fcm\n", prefix, gs->pos.x);
    printf("%s  y:% -10.5fcm\n", prefix, gs->pos.y);
    printf("%s  t:% -10.5f°\n", prefix, gs->pos.theta * 180. / M_PI);
    printf("%spos_u:\n", prefix);
    printf("%s  av:% -10.5fcm\n", prefix, sqrtf(gs->pos_u.a_var));
    printf("%s  bv:% -10.5fcm\n", prefix, sqrtf(gs->pos_u.b_var));
    printf("%s  an:% -10.5f°\n", prefix, gs->pos_u.a_angle * 180. / M_PI);
    printf("%s  tv:% -10.5f°\n", prefix, sqrtf(gs->pos_u.theta_var) * 180. / M_PI);
#endif
}

float rand_uniform(float min, float max){
    return ((double) rand() / ((double)RAND_MAX + 1)) * (max - min) + min;
}

int main(int argc, char *argv[]){
    {
    	printf("%i: Starting test: \"pos_uncertainty_mix\"\n", __LINE__);

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
        i1.pos_u.theta_var = powf(18.*M_PI/180., 2.);

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
        i2.pos.theta = M_PI/2.;
        i2.pos_u.theta_var = MINVARIANCE_THETA;

        printf("i2:\n");
        dump_gstatus(&i2, "  ");

        pos_uncertainty_mix(&i1, &i2, &o);

        printf("o:\n");
        dump_gstatus(&o, "  ");
    }

    {
#define IS_NEAR(a, b, eps) (fabsf((a) - (b)) <= fabsf(a)*(eps) || (a) < 1e-25f)
#define EXPECT_NEAR_PERCENT(a, b, eps, err) do {                                                    \
        if(!IS_NEAR(a, b, eps)) {                                                                   \
            printf("!!!!i=%i  not near: %g%%, %s\n", i, fabsf(((a) - (b))/((a)?(a):(b)))*100, err); \
            printf("!!!! \""#a"\" evaluates to: %g\n", (a));                                        \
            printf("!!!! \""#b"\" evaluates to: %g\n", (b));                                        \
            dump_gstatus(&in, "!!!!  ");                                                            \
            exit(1);                                                                                \
        }                                                                                           \
    } while(0)
#define EXPECT_EQUAL(a, b, err) do {                                            \
        if((a) != (b)) {                                                        \
            printf("!!!!i=%i  different: %g, %s\n", i, fabsf((a) - (b)), err);  \
            printf("!!!! \""#a"\" evaluates to: %g\n", (a));                    \
            printf("!!!! \""#b"\" evaluates to: %g\n", (b));                    \
            dump_gstatus(&in, "!!!!  ");                                        \
            exit(1);                                                            \
        }                                                                       \
    } while(0)

    	printf("%i: Starting test: \"gstatus2icovar;icovar2gstatus\"\n", __LINE__);

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

            EXPECT_EQUAL(in.pos.x, out.pos.x, "pos.x");
            EXPECT_EQUAL(in.pos.y, out.pos.y, "pos.y");
            EXPECT_EQUAL(in.pos.theta, out.pos.theta, "pos.theta");

            int j;
            s2DPosAtt pa;
            pa.frame = FRAME_PLAYGROUND;
            for(j = 0; j < 100; j++){
                pa.x = rand_uniform(0, 300);
                pa.y = rand_uniform(0, 200);
                pa.theta = rand_uniform(-M_PI, M_PI);

                s2DPAProbability ipap = pos_uncertainty_eval(&in, &pa);
                s2DPAProbability opap = pos_uncertainty_eval(&out, &pa);

                EXPECT_NEAR_PERCENT(ipap.xy_probability, opap.xy_probability, 2e-2f, "xy_prob");
                EXPECT_NEAR_PERCENT(ipap.theta_probability, opap.theta_probability, 2e-7f, "theta_prob");
            }
        }

        printf("OK!!!\n");
#undef IS_NEAR
#undef EXPECT_NEAR_PERCENT
#undef EXPECT_EQUAL
    }

    {
#define IS_NEAR(a, b, eps) (fabsf((a) - (b)) <= (eps))
#define EXPECT_NEAR(aa, bb, eps, err) do {                                      \
        if(!IS_NEAR(aa, bb, eps)) {                                             \
            printf("!!!!i=%i  not near: %g, %s\n", i, fabsf((aa) - (bb)), err); \
            printf("!!!! \""#aa"\" evaluates to: %g\n", (aa));                  \
            printf("!!!! \""#bb"\" evaluates to: %g\n", (bb));                  \
            printf("!!!! av:%.4f ; bv:%4f\n", x_var, y_var);                    \
            exit(1);                                                            \
        }                                                                       \
    } while(0)
#define THRESHOLD_VAR (1)
#define THRESHOLD_XY (1)
#define THRESHOLD_THETA (1e-1)

    	printf("%i: Starting test: \"covar2gstatus\"\n", __LINE__);

        int i;
        for(i = 0; i < 10000; i++){
            s2DPUncert_covar in;
            sGenericPosStatus out;

            float x_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            float y_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            while(fabsf(y_var - x_var) < 1e-2 * x_var){
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

    {
#define IS_NEAR(a, b, eps) (fabsf((a) - (b)) <= fabsf(a)*(eps) || (a) < 1e-25f)
#define EXPECT_NEAR_PERCENT(a, b, eps, err) do {                                                    \
        if(!IS_NEAR(a, b, eps)) {                                                                   \
            printf("!!!!i=%i  not near: %g%%, %s\n", i, fabsf(((a) - (b))/((a)?(a):(b)))*100, err); \
            printf("!!!! \""#a"\" evaluates to: %g\n", (a));                                        \
            printf("!!!! \""#b"\" evaluates to: %g\n", (b));                                        \
            dump_gstatus(&in, "!!!!  ");                                                            \
            exit(1);                                                                                \
        }                                                                                           \
    } while(0)
#define EXPECT_EQUAL(a, b, err) do {                                            \
        if((a) != (b)) {                                                        \
            printf("!!!!i=%i  different: %g, %s\n", i, fabsf((a) - (b)), err);  \
            printf("!!!! \""#a"\" evaluates to: %g\n", (a));                    \
            printf("!!!! \""#b"\" evaluates to: %g\n", (b));                    \
            dump_gstatus(&in, "!!!!  ");                                        \
            exit(1);                                                            \
        }                                                                       \
    } while(0)

    	printf("%i: Starting test: \"gstatus2covar;covar2gstatus\"\n", __LINE__);

        s2DPUncert_covar tmp;
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

            gstatus2covar(&in, &tmp);
            covar2gstatus(&tmp, &out);

            EXPECT_EQUAL(in.pos.x, out.pos.x, "pos.x");
            EXPECT_EQUAL(in.pos.y, out.pos.y, "pos.y");
            EXPECT_EQUAL(in.pos.theta, out.pos.theta, "pos.theta");

            int j;
            s2DPosAtt pa;
            pa.frame = FRAME_PLAYGROUND;
            for(j = 0; j < 100; j++){
                pa.x = rand_uniform(0, 300);
                pa.y = rand_uniform(0, 200);
                pa.theta = rand_uniform(-M_PI, M_PI);

                s2DPAProbability ipap = pos_uncertainty_eval(&in, &pa);
                s2DPAProbability opap = pos_uncertainty_eval(&out, &pa);

                EXPECT_NEAR_PERCENT(ipap.xy_probability, opap.xy_probability, 4e-3f, "xy_prob");
                EXPECT_EQUAL(ipap.theta_probability, opap.theta_probability, "theta_prob");
            }
        }

        printf("OK!!!\n");
#undef IS_NEAR
#undef EXPECT_NEAR_PERCENT
#undef EXPECT_EQUAL
    }

    {
#define IS_NEAR(a, b, eps) (fabsf((a) - (b)) <= fabsf(a)*(eps) || (a) < 1e-25f)
#define EXPECT_NEAR_PERCENT(a, b, eps, err) do {                                                    \
        if(!IS_NEAR(a, b, eps)) {                                                                   \
            printf("!!!!i=%i  not near: %g%%, %s\n", i, fabsf(((a) - (b))/((a)?(a):(b)))*100, err); \
            printf("!!!! \""#a"\" evaluates to: %g\n", (a));                                        \
            printf("!!!! \""#b"\" evaluates to: %g\n", (b));                                        \
            dump_gstatus(&out1, "!!!!1  ");                                                         \
            dump_gstatus(&out2, "!!!!2  ");                                                         \
            exit(1);                                                                                \
        }                                                                                           \
    } while(0)
#define EXPECT_EQUAL(a, b, err) do {                                            \
        if((a) != (b)) {                                                        \
            printf("!!!!i=%i  different: %g, %s\n", i, fabsf((a) - (b)), err);  \
            printf("!!!! \""#a"\" evaluates to: %g\n", (a));                    \
            printf("!!!! \""#b"\" evaluates to: %g\n", (b));                    \
            dump_gstatus(&out1, "!!!!1  ");                                     \
            dump_gstatus(&out2, "!!!!2  ");                                     \
            exit(1);                                                            \
        }                                                                       \
    } while(0)

    	printf("%i: Starting test: \"covar2gstatus;covar2icovar;icovar2gstatus\"\n", __LINE__);

        int i;
        for(i = 0; i < 10000; i++){
            s2DPUncert_covar in;
            s2DPUncert_icovar tmp;
            sGenericPosStatus out1, out2;
            memset(&out1, 0, sizeof(out1));
            memset(&out2, 0, sizeof(out2));

            // setting random linear position
            float x_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            float y_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            while(fabsf(y_var - x_var) < 1e-2 * x_var){
                y_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            }
            {
                if(x_var < y_var){
                    float tmp = x_var;
                    x_var = y_var;
                    y_var = tmp;
                }
            }
            float x_angle = rand_uniform(-M_PI, M_PI);
            float cxa = cosf(x_angle), sxa = sinf(x_angle);
            in.a = cxa*cxa*x_var + sxa*sxa*y_var;
            in.b = cxa*sxa*(x_var - y_var);
            in.c = cxa*cxa*y_var + sxa*sxa*x_var;
            in.x = rand_uniform(0, 300);
            in.y = rand_uniform(0, 200);

            // setting random angular position
            in.d = rand_uniform(MINVARIANCE_THETA, MAXVARIANCE_THETA);
            in.theta = rand_uniform(-M_PI, M_PI);

            covar2gstatus(&in, &out1);
            covar2icovar(&in, &tmp);
            icovar2gstatus(&tmp, &out2);

            EXPECT_EQUAL(out1.pos.x, out2.pos.x, "pos.x");
            EXPECT_EQUAL(out1.pos.y, out2.pos.y, "pos.y");
            EXPECT_EQUAL(out1.pos.theta, out2.pos.theta, "pos.theta");

            int j;
            s2DPosAtt pa;
            pa.frame = FRAME_PLAYGROUND;
            for(j = 0; j < 100; j++){
                pa.x = rand_uniform(0, 300);
                pa.y = rand_uniform(0, 200);
                pa.theta = rand_uniform(-M_PI, M_PI);

                s2DPAProbability ipap = pos_uncertainty_eval(&out1, &pa);
                s2DPAProbability opap = pos_uncertainty_eval(&out2, &pa);

                EXPECT_NEAR_PERCENT(ipap.xy_probability, opap.xy_probability, 5e-2f, "xy_prob");
                EXPECT_NEAR_PERCENT(ipap.theta_probability, opap.theta_probability, 3e-7f, "theta_prob");
            }
        }

        printf("OK!!!\n");
#undef IS_NEAR
#undef EXPECT_NEAR_PERCENT
#undef EXPECT_EQUAL
    }

    {
#define IS_NEAR(a, b, eps) (fabsf((a) - (b)) <= (eps))
#define EXPECT_NEAR(aa, bb, eps, err) do {                                       \
        if(!IS_NEAR(aa, bb, eps)) {                                              \
            printf("!!!!i=%i  not near: %g, %s\n", i, fabsf((aa) - (bb)), err);  \
            printf("!!!! \""#aa"\" evaluates to: %g\n", (aa));                   \
            printf("!!!! \""#bb"\" evaluates to: %g\n", (bb));                   \
            printf("!!!! av:%.4f ; bv:%4f\n", x_var, y_var);                     \
            exit(1);                                                             \
        }                                                                        \
    } while(0)
#define THRESHOLD_VAR (1)
#define THRESHOLD_XY (1)
#define THRESHOLD_THETA (1e-1)

    	printf("%i: Starting test: \"covar2icovar\"\n", __LINE__);

        int i;
        for(i = 0; i < 10000; i++){
            s2DPUncert_covar in;
            s2DPUncert_icovar out;

            float x_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            float y_var = rand_uniform(MINVARIANCE_XY, MAXVARIANCE_XY);
            while(fabsf(y_var - x_var) < 1e-2 * x_var){
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
            in.d = 10.;

            covar2icovar(&in, &out);

            float m11 = in.a*out.a + in.b*out.b;
            float m12 = in.a*out.b + in.b*out.c;
            float m21 = in.b*out.a + in.c*out.b;
            float m22 = in.b*out.b + in.c*out.c;
            float m33 = in.d*out.d;

            EXPECT_NEAR(m11, 1.f, 2e-4f, "m11");
            EXPECT_NEAR(m12, 0.f, 2e-4f, "m12");
            EXPECT_NEAR(m21, 0.f, 2e-4f, "m21");
            EXPECT_NEAR(m22, 1.f, 2e-4f, "m22");
            EXPECT_NEAR(m33, 1.f, 1e-10f, "m33");
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
