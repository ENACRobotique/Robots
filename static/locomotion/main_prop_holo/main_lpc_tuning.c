#include <debug.h>
#include <gpio.h>
#include <ime.h>
#include <pwm.h>
#include <sys_time.h>
#include <uart0.h>
#include <stdio.h>

#include "tools.h"
#include "pins.h"
#include "params.h"
#include "messages.h"
#include "position_controller.h"
#include "trajectory_manager.h"
#include "shared/botNet_core.h"

/*
 * pins usage and mapping (board Rev3):
 *      UART0-TXD   P0.0    EXT1.1      TX_LPC              !PWM1
 *      UART0-RXD   P0.1    EXT1.2      RX_LPC              !PWM3 ~EINT0
 *      I²C0-SCL    P0.2    EXT1.3      SCL_LPC
 *      I²C0-SDA    P0.3    EXT1.4      SDA_LPC             ~EINT1
 *      GPIO-OUT    P0.4    EXT1.5      DIR_POD2_LPC
 *      GPIO-OUT    P0.5    EXT1.6      DIR_POD1_LPC
 *      GPIO-OUT    P0.6    EXT1.7      DIR_POD3_LPC
 *      PWM2        P0.7    EXT1.8      PWM_POD1_LPC        ~EINT2
 *      PWM4        P0.8    EXT1.9      PWM_POD2_LPC        !UART1
 *      PWM6        P0.9    EXT1.10     PWM_POD3_LPC        !UART1 ~EINT3
 *      GPIO-IN     P0.12   EXT1.13     CHA_POD1_LPC
 *      EINT2       P0.15   EXT1.16     CHB_POD1_LPC        ~UART1
 *      EINT0       P0.16   EXT1.17     CHB_POD2_LPC
 *      GPIO-IN     P0.17   EXT1.18     CHA_POD2_LPC
 *      EINT3       P0.20   EXT1.21     CHB_POD3_LPC
 *      GPIO-IN     P0.21   EXT1.22     CHA_POD3_LPC
 *      GPIO-IN     P0.28   EXT2.1      SW1_LPC
 *      GPIO-IN     P0.29   EXT2.2      SW2_LPC
 *      GPIO-IN     P0.30   EXT2.3      SW3_LPC
 *      GPIO-OUT    P0.31   EXT2.4      LED
 *      GPIO-OUT    P1.24   EXT2.13     LED
 *      DEBUG       P1.26   EXT2.15
 *      DEBUG       P1.27   EXT2.16
 *      DEBUG       P1.28   EXT2.17
 *      DEBUG       P1.29   EXT2.18
 *      DEBUG       P1.30   EXT2.19
 *
 * for more details, see Features2Pins.txt file
 */

/*
 *      y axis
 *         ^
 *    P2   #   P1
 *     \   #   /
 *      \  #  /
 *       \ # /
 *        \#/
 *         o######### > x axis
 *         |
 *         |
 *         |
 *         |
 *         P3
 */

/* from theoretical data
 M_rob2pods =
 -5.00000000000000e-01   8.66025403784439e-01   1.34580000000000e+01
 -5.00000000000000e-01  -8.66025403784439e-01   1.34580000000000e+01
 1.00000000000000e+00  -1.83697019872103e-16   1.34580000000000e+01
 */
const int32_t mat_rob2pods[NB_PODS][NB_SPDS] = {
        { -0.5 * dMSHIFT, 0.866025403784439 * dMSHIFT, D2I(13.458) * dMoRSHIFT },
        { -0.5 * dMSHIFT, -0.866025403784439 * dMSHIFT, D2I(13.458) * dMoRSHIFT },
        { 1. * dMSHIFT, 0 * dMSHIFT, D2I(13.458) * dMoRSHIFT }
};

#define TUNING 1

/**
 * 4/pi times:
 *
 *      ^
 *      |
 *   1  |\              /
 *      | \            /
 *      |  \          /
 *   0  |___\________/__2*PI
 *      |    \      /
 *      |     \    /
 *      |      \  /
 *  -1  |       \/
 *      |
 */
double lincos(double theta){
    double alpha = theta >= 0 ? fmod(theta, 2*PI) : fmod(theta, 2*PI) + 2*PI;
    double v;

    if(alpha < PI){
        v = 1. - alpha*2./PI;
    }
    else{
        v = -3 + alpha*2./PI;
    }

    return 4.*v/PI; // factor to get a 1-based primitive
}

double linsin(double theta){
    return lincos(PI/2. - theta);
}

/**
 * 2/pi times:
 *
 *      ^
 *      |
 *   1  |_____
 *      |     |
 *      |     |
 *   0  |_____|_____2*PI
 *      |     |     |
 *      |     |     |
 *  -1  |     |_____|
 *      |
 */
double cstsin(double theta){
    double alpha = theta >= 0 ? fmod(theta, 2*PI) : fmod(theta, 2*PI) + 2*PI;
    double v;

    if(alpha < PI){
        v = 1.;
    }
    else{
        v = -1.;
    }

    return 2.*v/PI; // factor to get 1-based primitive
}

double cstcos(double theta){
    return cstsin(PI/2. - theta);
}

int main() {
    gpio_init_all();
    debug_leds_init();
    debug_switches_init();
    sys_time_init();
    pwm_init(0, PWM_RANGE); // frequency of the generated pwm signal: equal f_osc/((prescaler + 1)*range)
    uart0_init(115200);

    unsigned int prevControl_us = micros();
    unsigned int prevLed_ms = millis();
    unsigned int time_ms, time_us;
    int state = 0;

#if TUNING == 0
    motor_t mots[3];
    motors_init(mots);
    encoder_t encs[3];
    encoders_init(encs, mots);
    speed_controller_t scs[3];
    int i;
    for(i = 0; i < 3; i++) {
        spdctlr_init(&scs[i], &encs[i]);
    }

    int sps[3] = {0, 0, 0};
#elif TUNING == 1
    position_controller_t pc;
    posctlr_init(&pc, mat_rob2pods);
    int x_sp = 0, y_sp = 0, theta_sp = 0;

    int sps[3] = {0, 0, 0};

    MT_VEC spd_cmd_rob = MT_V_INITS(NB_SPDS, VEC_SHIFT); // (Vx_cmd, Vy_cmd, Oz_pv) (in [IpP<<SHIFT]x[IpP<<SHIFT]x[radpP<<(RAD_SHIFT+SHIFT)])
    MT_VEC spd_cmd_pods = MT_V_INITS(NB_PODS, VEC_SHIFT); // (V1_cmd, V2_cmd, V3_cmd) (in IpP << SHIFT)
#endif

    global_IRQ_enable();

    while (1) {
        sys_time_update();

#if TUNING == 0
        time_us = micros();
        if (time_us - prevControl_us >= USpP) {
            prevControl_us = time_us;

            {
                double v;
                double omega = 2.*PI/3.5; // (rad/s)
                double theta = omega*(double)time_us/1e6; // (rad)
                double a = 40.;

#if 0
                v = sin(theta);
#else
                // more representative, piecewise linear speed
                v = linsin(theta);
#endif

                sps[0] = iDpS2IpP( a/2. *v);
                sps[1] = iDpS2IpP( a/2. *v);
                sps[2] = iDpS2IpP(-a    *v);
            }

            for(i = 0; i < 3; i++) {
                encoder_update(&encs[i]);
                spdctlr_update(&scs[i], sps[i]);
                motor_update(&mots[i], spdctlr_get(&scs[i]));
            }

            printf("%u, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n",
                    time_us,
                    encs[0].nbticks_cache, encs[1].nbticks_cache, encs[2].nbticks_cache,
                    scs[0].cmd_cache, scs[1].cmd_cache, scs[2].cmd_cache,
                    sps[0], sps[1], sps[2],
                    scs[0].pid.sum, scs[1].pid.sum, scs[2].pid.sum);
        }
#elif TUNING == 1
        time_us = micros();
        if (time_us - prevControl_us >= USpP) { // takes ~290µs!
            prevControl_us = time_us;

            posctlr_begin_update(&pc);

            {
                // lissajous curve for X/Y, why not?
                double p = 3.; // must be integer
                double q = 4.; // must be integer
                double a = 20.; // (cm)
                double b = 4.; // (cm)
                double phi = 0; // (rad) (must be < PI/(2*p))

                double omega = 2.*PI/15.; // (rad/s)
                double theta = omega*(double)time_us/1e6; // (rad)

                // just a linear sinus for orientation
                double c = 15. * PI / 180.; // (rad)
                double omegap = 2.*PI/4.; // (rad/s)
                double thetap = omegap*(double)time_us/1e6; // (rad)

#if 0
                sps[0] = isDpS2IpP(omega*a*p*cos(p*theta));
                sps[1] = isDpS2IpP(omega*b*q*cos(q*theta + phi));
                sps[2] = iROUND(omegap*c*cos(thetap) * SpP * dASHIFT);
#else
                // more representative, piecewise-linear continuous linear speed on each axis and piecewise-linear continuous angular speed
                sps[0] = isDpS2IpP(omega*a*p*lincos(p*theta));
                sps[1] = isDpS2IpP(omega*b*q*lincos(q*theta + phi));
                sps[2] = iROUND(omegap*c*lincos(thetap) * SpP * dASHIFT);
#endif
            }

            x_sp += sps[0];
            y_sp += sps[1];
            theta_sp += sps[2];

            posctlr_end_update(&pc, x_sp, y_sp, theta_sp, sps[0], sps[1], sps[2]);

            int v1_sp, v2_sp, v3_sp;
            {   // compute setpoint V1,V2,V3
                spd_cmd_rob.ve[0] = sps[0];
                spd_cmd_rob.ve[1] = sps[1];
                spd_cmd_rob.ve[2] = sps[2];

                mt_mv_mlt(&pc.M_spds_rob2pods, &spd_cmd_rob, &spd_cmd_pods);

                v1_sp = spd_cmd_pods.ve[0] >> VEC_SHIFT;
                v2_sp = spd_cmd_pods.ve[1] >> VEC_SHIFT;
                v3_sp = spd_cmd_pods.ve[2] >> VEC_SHIFT;
            }

            printf("%u, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %u\n",
                    time_us,
                    x_sp, y_sp, theta_sp,
                    pc.x, pc.y, pc.theta,
                    v1_sp, v2_sp, v3_sp,
                    pc.encs[0].nbticks_cache, pc.encs[1].nbticks_cache, pc.encs[2].nbticks_cache,
                    micros() - time_us);
        }
#endif

        time_ms = millis();
        if ((time_ms - prevLed_ms) > 200) {
            prevLed_ms = time_ms;
            state ^= 1;
            gpio_write(1, 24, state);
        }
    }
}
