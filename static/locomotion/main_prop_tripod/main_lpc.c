#include <debug.h>
#include <gpio.h>
#include <ime.h>
#include <pins.h>
#include <params.h>
#include <pwm.h>
#include <sys_time.h>
#include <tools.h>
#include "messages.h"
#include "trajectory_controller.h"
#include "shared/botNet_core.h"

/*
 * pins usage and mapping (board Rev2):
 *      UART0-TXD   P0.0    EXT1.1      TX_LPC              !PWM1
 *      UART0-RXD   P0.1    EXT1.2      RX_LPC              !PWM3 ~EINT0
 *      I²C0-SCL    P0.2    EXT1.3      SCL_LPC
 *      I²C0-SDA    P0.3    EXT1.4      SDA_LPC             ~EINT1
 *      GPIO-OUT    P0.4    EXT1.5      DIR_POD1_LPC
 *      GPIO-OUT    P0.5    EXT1.6      DIR_POD2_LPC
 *      GPIO-OUT    P0.6    EXT1.7      DIR_POD3_LPC
 *      PWM2        P0.7    EXT1.8      PWM_POD1_LPC        ~EINT2
 *      PWM4        P0.8    EXT1.9      PWM_POD2_LPC        !UART1
 *      PWM6        P0.9    EXT1.10     PWM_POD3_LPC        !UART1 ~EINT3
 *      EINT1       P0.14   EXT1.15     CHB_POD1_LPC        !I²C1 ~UART1
 *      GPIO-IN     P0.15   EXT1.16     CHA_POD1_LPC
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

/* from theoric data
M_rob2pods =
  -5.00000000000000e-01   8.66025403784439e-01   1.55000000000000e+01
  -5.00000000000000e-01  -8.66025403784439e-01   1.55000000000000e+01
   1.00000000000000e+00  -1.83690953073357e-16   1.55000000000000e+01
*/
#define dMSHIFT ((double)(1 << MT_MAT_SHIFT))

const int32_t mat_rob2pods[3][3] = {
    {-0.5 * dMSHIFT,  0.866025403784439 * dMSHIFT, 15.5 * dMSHIFT},
    {-0.5 * dMSHIFT, -0.866025403784439 * dMSHIFT, 15.5 * dMSHIFT},
    { 1.  * dMSHIFT,  0                 * dMSHIFT, 15.5 * dMSHIFT}
};

int main() {
    //// Initialization
    gpio_init_all();
    // Debug
    debug_leds_init();
    // Small switches
    debug_switches_init();
    // LED
    gpio_output(1, 24);   // writes to output {1,24}
    gpio_output(0, 31);  // writes to output {0,31}
    // Time
    sys_time_init();
    // PWM
    pwm_init(0, PWM_RANGE); // frequency of the generated pwm signal: equal f_osc/((prescaler + 1)*range)
    // Trajectory
    trajectory_controller_t traj_ctl;
    trajctl_init(&traj_ctl, mat_rob2pods);

    global_IRQ_enable();

    //// Global variables
    unsigned int prevControl = millis();
    int ret;
    sMsg inMsg = {{0}};//, outMsg = {{0}};

    while (1) { // ############## Loop ############################################
        sys_time_update();

        // Reception and processing of the message
        ret = bn_receive(&inMsg);
        if(ret > 0){
//            process_msg(&inMsg, &outMsg);// TODO
        }

        if (micros() - prevControl >= PER_ASSER) {
            prevControl += PER_ASSER;

            // Control trajectory
            trajctl_update(&traj_ctl);
        }
    } // ############## End loop ############################################
}



/*
 * ############## Remarks ####################
 * 1) May be used isRound to have more precision
 *
 */
