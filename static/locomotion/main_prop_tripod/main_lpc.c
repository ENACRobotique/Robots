#include <debug.h>
#include <encoders.h>
#include <gpio.h>
#include <ime.h>
#include <motors.h>
#include <param.h>
#include <pwm.h>
#include <sys_time.h>
#include <tools.h>

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
 *      EINT3       P0.20   EXT1.21     CHA_POD3_LPC
 *      GPIO-IN     P0.21   EXT1.22     CHB_POD3_LPC
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

int main() {
    //// Initialization
    gpio_init_all();
    // Debug
    debug_init();
    // Small switch
    switchs_init();
    // LED
    gpio_output(1, 24);   // writes to output {1,24}
    gpio_output(0, 31);  // writes to output {0,31}
    // Time
    sys_time_init();
    // Init PWM
    pwm_init(0, PWM_RANGE); // frequency of the generated pwm signal: equal f_osc/((prescaler + 1)*range)
    // Init motors
    motors_init();
    // Init encoders
    encoders_init();

    global_IRQ_enable();

    unsigned int prevControl = millis();
    while (1) { // ############## Loop ############################################
        sys_time_update();

        if (micros() - prevControl >= 20000) {
            prevControl += 20000;

            // Get direction
            // TODO

            // Get speed consign
            // TODO

            // Motor control
            // TODO
        }
    } // ############## End loop ############################################
}
