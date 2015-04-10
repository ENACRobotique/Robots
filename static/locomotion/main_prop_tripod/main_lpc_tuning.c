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

#define TUNING 0

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

    unsigned int prevTraj_us = micros();
    int spd_state = 0;
    int sps[3] = {0, 0, 0};
#elif TUNING == 1
    position_controller_t pc;
    posctlr_init(&pc, mat_rob2pods);
    int i;
    int x_sp = 0, y_sp = 0, theta_sp = 0;

    unsigned int prevTraj_us = micros();
    int spd_state = 0;
    int sps[3] = {0, 0, 0};
#endif

    global_IRQ_enable();

    while (1) {
        sys_time_update();

#if TUNING == 0
        time_us = micros();
        if (time_us - prevControl_us >= USpP) {
            prevControl_us = time_us;

            for(i = 0; i < 3; i++) {
                encoder_update(&encs[i]);
                spdctlr_update(&scs[i], sps[i]);
                motor_update(&mots[i], spdctlr_get(&scs[i]));
            }

            printf("%u, %i, %i, %i, %i, %i, %i, %i, %i, %i\n",
                    time_us,
                    encs[0].nbticks_cache, encs[1].nbticks_cache, encs[2].nbticks_cache,
                    scs[0].cmd_cache, scs[1].cmd_cache, scs[2].cmd_cache,
                    sps[0], sps[1], sps[2]);
        }

        if (time_us - prevTraj_us >= 2000000ull) {
            prevTraj_us = time_us;

            spd_state = (spd_state + 1)%4;

            switch(spd_state) {
            default:
            case 0:
                sps[0] = 0;
                sps[1] = 0;
                sps[2] = 0;
                break;
            case 1:
                sps[0] = iDpS2IpP(-8.);
                sps[1] = iDpS2IpP(-8.);
                sps[2] = iDpS2IpP(16.);
                break;
            case 2:
                sps[0] = 0;
                sps[1] = 0;
                sps[2] = 0;
                break;
            case 3:
                sps[0] = iDpS2IpP(  8.);
                sps[1] = iDpS2IpP(  8.);
                sps[2] = iDpS2IpP(-16.);
                break;
            }
        }

#elif TUNING == 1
        time_us = micros();
        if (time_us - prevControl_us >= USpP) {
            posctlr_begin_update(&pc);

            {
                sps[0] = isDpS2IpP(50.*sin(2.*PI*(double)time_us/4000000.));;
                sps[1] = 0;
                sps[2] = 0;
            }

            x_sp += sps[0];
            y_sp += sps[1];
            theta_sp += sps[2];

            posctlr_end_update(&pc, x_sp, y_sp, theta_sp, sps[0], sps[1], sps[2]);

            printf("%u, %i, %i, %i, %i, %i, %i\n",
                    time_us,
                    x_sp, y_sp, theta_sp,
                    pc.x, pc.y, pc.theta);

            prevControl_us = time_us;
        }

//        if (time_us - prevTraj_us >= 2000000ull) {
//            prevTraj_us = time_us;
//
//            spd_state = (spd_state + 1)%4;
//
//            switch(spd_state) {
//            default:
//            case 0:
//                sps[0] = 0;
//                sps[1] = 0;
//                sps[2] = 0;
//                break;
//            case 1:
//                sps[0] = isDpS2IpP(-20.);
//                sps[1] = 0;
//                sps[2] = iROUND( 20.*PI/180.*SpP*dASHIFT);
//                break;
//            case 2:
//                sps[0] = 0;
//                sps[1] = 0;
//                sps[2] = 0;
//                break;
//            case 3:
//                sps[0] = isDpS2IpP( 20.);
//                sps[1] = 0;
//                sps[2] = iROUND(-20.*PI/180.*SpP*dASHIFT);
//                break;
//            }
//        }
#endif

        time_ms = millis();
        if ((time_ms - prevLed_ms) > 200) {
            prevLed_ms = time_ms;
            state ^= 1;
            gpio_write(1, 24, state);
        }
    }
}
