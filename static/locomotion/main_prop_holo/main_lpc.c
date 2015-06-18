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
#include "bn_intp.h"
#include "roles.h"

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

int main() {
    //// Initialization
    gpio_init_all();
    debug_leds_init();
    debug_switches_init();
    sys_time_init();
    pwm_init(0, PWM_RANGE); // frequency of the generated pwm signal: equal f_osc/((prescaler + 1)*range)

    trajectory_manager_t traj_mngr;
    trajmngr_init(&traj_mngr, mat_rob2pods);

    bn_attach(E_ROLE_SETUP, role_setup);
    bn_intp_install();
    bn_init();

    //// Global variables
    unsigned int prevControl_us = micros();
    unsigned int prevLed_ms = millis();
    unsigned int prevPos_ms = millis();
    unsigned int time_ms, time_us;
    int ledState = 0;
    int ret;
    int firstPosReceived = 0;
    sMsg inMsg = { { 0 } }, outMsg = { { 0 } };

    // FIXME need to have a first loop here to wait for time synchronization

    global_IRQ_enable();

    while (1) {
        sys_time_update();

        // Reception and processing of the message
        ret = bn_receive(&inMsg);
        if (ret > 0) {
            switch (inMsg.header.type) {
            case E_TRAJ_ORIENT_EL: // Get the new step of a trajectory
                trajmngr_new_traj_el(&traj_mngr, &inMsg.payload.trajOrientEl);
                break;
            case E_GENERIC_POS_STATUS:
                switch(inMsg.payload.genericPosStatus.prop_status.action){
                case PROP_SETPOS:
                    trajmngr_set_pos(&traj_mngr, &inMsg.payload.genericPosStatus);
                    firstPosReceived = 1;
                    break;
                case PROP_MIXPOS:
                    trajmngr_mix_pos(&traj_mngr, &inMsg.payload.genericPosStatus);
                    break;
                default:
                    break;
                }
                break;
            case E_PROP_STOP:
                trajmngr_stop(&traj_mngr);
                break;
            }
        } // End: if(ret > 0)

        // Automatic control
        time_us = micros();
        if (time_us - prevControl_us >= USpP) {
            if (time_us - prevControl_us > 1.5*USpP) {
                // If there is too much delay we skip the step
                trajmngr_reset(&traj_mngr);
            }
            else {
                // Control trajectory
                trajmngr_update(&traj_mngr);
            }

            prevControl_us = time_us;
        }

        time_ms = millis();
        // Periodic position send
        if (time_ms - prevPos_ms >= 100 && firstPosReceived) {
            prevPos_ms = time_ms;

            memset(&outMsg, 0, sizeof(outMsg));

            outMsg.header.type = E_GENERIC_POS_STATUS;
            outMsg.header.size = sizeof(outMsg.payload.genericPosStatus);

            trajmngr_get_pos_status(&traj_mngr, &outMsg.payload.genericPosStatus);

            role_send(&outMsg, ROLEMSG_PRIM_POS);
        }

        time_ms = millis();
        if ((time_ms - prevLed_ms) > 200) {
            prevLed_ms = time_ms;
            ledState ^= 1; // toggle state (led blinking)
            gpio_write(1, 24, ledState);
        }
    }
}
