#include <lpc214x.h>
#include <gpio.h>
#include <sys_time.h>
#include <ime.h>
#include <pwm.h>
#include "param.h"
#include "debug.h"
#include "motor.h"
#include "encoder.h"
#include "tools.h"

/*
 * pins usage and mapping (board Rev?):
 *      UART0-TXD   P0.0    EXT1.1      TX_LPC              !PWM1
 *      UART0-RXD   P0.1    EXT1.2      RX_LPC              !PWM3 ~EINT0
 *      I²C0-SCL    P0.2    EXT1.3      SCL_LPC
 *      I²C0-SDA    P0.3    EXT1.4      SDA_LPC             ~EINT1
 *      GPIO-OUT    P0.4    EXT1.5      SD1_POD1_LPC
 *      GPIO-OUT    P0.5    EXT1.6      SD1_POD2_LPC
 *      GPIO-OUT    P0.6    EXT1.7      SD1_POD3_LPC
 *      PWM2        P0.7    EXT1.8      PWM_POD1_LPC        ~EINT2
 *      PWM4        P0.8    EXT1.9      PWM_POD2_LPC        !UART1
 *      PWM6        P0.9    EXT1.10     PWM_POD3_LPC        !UART1 ~EINT3
 *      GPIO-OUT    P0.10   EXT1.11     SD2_POD1_LPC        ~UART1
 *      GPIO-OUT    P0.11   EXT1.12     SD2_POD2_LPC        !I²C1 ~UART1
 *      GPIO-OUT    P0.12   EXT1.13     SD2_POD3_LPC        ~UART1
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

volatile float speedCons; // Consign wheel speed in inc/T
float speed_mes; // Wheel speed in inc/T
float speedErr; // Speed command for motor
volatile eMotorDir dirCons; // Consign wheel direction
int timeStartLoop;
unsigned int prevLEDMillis = 0;
int stateLED = 0;

int main() {
    dirCons = 0;
    int pwmCmd = 200;	// Value between 0 and pwm_range
    eMotorOperation motOp = Drive;
#ifdef ENCODER
    int prevTime = 0;
    int irqCptRef = 0;
    int irqCptPrev = 0;
#endif

    //// Initialization
    gpio_init_all();
    // Command bridges
    gpio_output(BK_SD2, PIN_SD2); // IN2 out
    gpio_output(BK_SD1, PIN_SD1); // IN1 out
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
    pwm_enable(1, pwmCmd /* between 0 and range specified in pwm_init */);
    pwm_enable(2, pwmCmd);
    // init encoders
    encoders_init();

    global_IRQ_enable();

    timeStartLoop = millis();

    while (1) { // ############## Loop ############################################
        sys_time_update();

//		if((millis() - timeStartLoop) >= 0){
//			timeStartLoop += T_ASSER;
//			timeStartLoop = millis();
//			if((millis() - timeStartLoop) > T_ASSER/2){
//				timeStartLoop = millis();
//				continue;
//			}

        // Get direction
        dirCons = READ_DIR_ASKED;

        // Get motor operation
        switch (gpio_read(BK_SWTCH7, PIN_SWTCH7) + gpio_read(BK_SWTCH8, PIN_SWTCH8)) {
        case 0:
            motOp = Drive;
            break;
        case 1:
            motOp = FreeWheel;
            break;
        case 2:
            motOp = Braking;
            break;
        case 3: // Unattributed
            break;
        default:
            DEBUG_5_ON;
            break;
        }

#ifdef ENCODER
        // Get number of IRQ
        global_IRQ_disable();
        irqCptRef = irqCpt;
        irqCpt = 0;
        global_IRQ_enable();
#endif

        // Get speed consign
        int c = (gpio_read(BK_SWTCH1, PIN_SWTCH1) + 2 * gpio_read(BK_SWTCH2, PIN_SWTCH2) + 4 * gpio_read(BK_SWTCH3, PIN_SWTCH3) + 8 * gpio_read(BK_SWTCH4, PIN_SWTCH4));
        speedCons = mPerS2IncPerT(c * MAX_SPEED / 15);
        if (gpio_read(BK_SWTCH2, PIN_SWTCH2) == 1)
            DEBUG_3_ON;
        else
            DEBUG_3_OFF;

#ifdef ENCODER
        // Get speed measured
        if((millis() - prevTime) > T_ASSER) {
            speed_mes = (irqCptRef - irqCptPrev) / (millis() - prevTime); // inc/period
            prevTime = millis();
        }

        // Calculation of speed command & pwmCmd
        speedErr = (speed_mes - speedCons);
        pwmCmd = PWM_RANGE*ROUND(speedErr/mPerS2IncPerT(MAX_SPEED));
        if(pwmCmd >= PWM_RANGE)
        pwmCmd = PWM_RANGE;
        else if(pwmCmd <= 0)
        pwmCmd = 0;
#endif

        // Control motor
#ifdef ENCODER
        controlMotor(pwmCmd, dirCons, Drive);
#else
        controlMotor(c * PWM_RANGE / 15, dirCons, motOp);
#endif
//		} // if((millis() - timeStartLoop) >= 0)

    } // ############## End loop ############################################
}
