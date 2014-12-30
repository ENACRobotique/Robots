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



/* The H bridge
 *                      |+
 *           ___________|___________
 *          |                       |
 *          |                       |
 *          _/ Q_H1                 _/ Q_H2
 *          |                       |
 *          |      +--> Trigo       |
 *          |___Mot1        Mot2____|
 *          |     <--+ NoTrigo      |
 *          |                       |
 *          _/ Q_L1                 _/ Q_L2
 *          |                       |
 *          |_______________________|
 *                      |
 *                      |-
 */

/*
 * pins usage (board Rev1):
 *      UART0
 *          TXD0    P0.0    EXT1.1      TX_LPC              !PWM1
 *          RXD0    P0.1    EXT1.2      RX_LPC              !PWM3 ~EINT0
 *      I²C0
 *          SCL0    P0.2    EXT1.3      SCL_LPC
 *          SDA0    P0.3    EXT1.4      SDA_LPC             ~EINT1
 *      PWM*
 *          PWM2    P0.7    EXT1.8      PWM1_LPC            ~EINT2
 *          PWM5    P0.21   EXT1.22     PWM2_LPC
 *      EINT*
 *          EINT0   P0.16   EXT1.17     CHNL_I_LPC
 *          EINT1   P0.14   EXT1.15     CHNL_A_LPC          !I²C1 ~UART1
 *          EINT3   P0.20   EXT1.21     CHNL_B_LPC
 *      GPIO*
 *          OUT     P0.6    EXT1.7      SD1_LPC
 *          OUT     P0.8    EXT1.9      LED_1               !UART1
 *          IN/OUT  P0.9    EXT1.10     pull-up SCL_LPC     !UART1
 *          IN/OUT  P0.10   EXT1.11     pull-up SDA_LPC     ~UART1
 *          IN      P0.11   EXT1.12     SW3_LPC             !I²C1 ~UART1
 *          IN      P0.12   EXT1.13     SW2_LPC             ~UART1
 *          IN      P0.13   EXT1.14     SW1_LPC             ~UART1
 *          OUT     P0.30   EXT2.3      SD2_LPC             ~EINT3
 *          OUT     P0.31   EXT2.4      LED_3
 *          OUT     P1.24   EXT2.13     LED3
 *      DEBUG
 *                  P1.26   EXT2.15
 *                  P1.27   EXT2.16
 *                  P1.28   EXT2.17
 *                  P1.29   EXT2.18
 *                  P1.30   EXT2.19
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
		// Bootstrap
		gpio_output(BK_BSTRP1, PIN_BSTRP1);
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
#ifdef ENCODER
		// External interrupts
		// rising edge channel A
	    eint_disable(EINT0);
	    eint_assign(EINT0_P0_16); // FIXME use the right assignation
	    eint_mode(EINT0, EINT_RISING_EDGE);
	    eint_register(EINT0, isr_eint0, 2);
	    eint_enable(EINT0);
	    // rising edge channel B
	    eint_disable(EINT3);
	    eint_assign(EINT3_P0_20); // FIXME use the right assignation
	    eint_mode(EINT3, EINT_RISING_EDGE);
	    eint_register(EINT3, isr_eint3, 3);
	    eint_enable(EINT3);

		// Enable IRQ
		global_IRQ_enable();
#endif

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
			switch(gpio_read(BK_SWTCH7, PIN_SWTCH7) + gpio_read(BK_SWTCH8, PIN_SWTCH8)){
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
			int c = (gpio_read(BK_SWTCH1, PIN_SWTCH1) + 2*gpio_read(BK_SWTCH2, PIN_SWTCH2) + 4*gpio_read(BK_SWTCH3, PIN_SWTCH3) + 8 *gpio_read(BK_SWTCH4, PIN_SWTCH4));
			speedCons = mPerS2IncPerT(c * MAX_SPEED/15);
			if(gpio_read(BK_SWTCH2, PIN_SWTCH2) == 1)
				DEBUG_3_ON;
			else
				DEBUG_3_OFF;

#ifdef ENCODER
			// Get speed measured
			if((millis() - prevTime) > T_ASSER){
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
			controlMotor(c*PWM_RANGE/15, dirCons, motOp);
#endif
//		} // if((millis() - timeStartLoop) >= 0)

	} // ############## End loop ############################################
}
