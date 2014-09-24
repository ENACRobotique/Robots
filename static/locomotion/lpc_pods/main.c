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



volatile float speedCons; // Consign wheel speed in inc/T
float speed_mes; // Wheel speed in inc/T
float speedCmd; // Speed command for motor
volatile int dirCons; // Consign wheel direction
int timeStartLoop;
unsigned int prevLEDMillis = 0;
int stateLED = 0;


int main() {
	dirCons = 0;
	int pwmCmd = 200;	// Value between 0 and pwm_range
#ifdef ENCODER
	int prevTime = 0;
	int irqCptRef = 0;
	int irqCptPrev = 0;
#endif

	//// Initialization
		gpio_init_all();
		// Command bridges
		gpio_output(BK_IN2, PIN_IN2); // IN2 out
		gpio_output(BK_IN1, PIN_IN1); // IN1 out
		// Debug
		gpios_debg_output();
		// Small switch
		gpio_input(BK_SWTCH1, PIN_SWTCH1);
		gpio_input(BK_SWTCH2, PIN_SWTCH2);
		gpio_input(BK_SWTCH3, PIN_SWTCH3);
		gpio_input(BK_SWTCH4, PIN_SWTCH4);
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
		// Init IRQ
		global_IRQ_disable();
		eint_init(isr_eint0 /* rising edge channel A */, isr_eint3 /* rising edge channel B */);
		global_IRQ_enable();
#endif

	timeStartLoop = millis();

	while (1) { // ############## Loop ############################################
		sys_time_update();

		if((millis() - timeStartLoop) >= 0){
//			timeStartLoop += T_ASSER;
			timeStartLoop = millis();
			if((millis() - timeStartLoop) > T_ASSER/2){
				timeStartLoop = millis();
				continue;
			}
			// Get direction
			#ifdef DVLPT_BOARD
			dirCons = READ_DIR_ASKED;

			#else
			// TODO
			#endif

#ifdef ENCODER
			// Get number of IRQ
			global_IRQ_disable();
			irqCptRef = irqCpt;
			irqCpt = 0;
			global_IRQ_enable();
#endif

			// Get speed consign
			#ifdef DVLPT_BOARD
			int c = (3*gpio_read(BK_SWTCH3, PIN_SWTCH3) + 4*gpio_read(BK_SWTCH4, PIN_SWTCH4));
			speedCons = mPerS2IncPerT(c * MAX_SPEED/7);
			if(c >= 3 && c < 4) blindLED(BK_GREEN_LED_BOARD, PIN_GREEN_LED_BOARD, 1000, 400, 0);
			if(c >= 4) blindLED(BK_ORANGE_LED_BOARD, PIN_ORANGE_LED_BOARD, 1000, 400, 0);
			#else
			// TODO
			#endif

#ifdef ENCODER
			// Get speed measured
			if((millis() - prevTime) > T_ASSER){
				speed_mes = (irqCptRef - irqCptPrev) / (millis() - prevTime); // inc/period
				prevTime = millis();
			}

			// Calculation of speed command & pwmCmd
			speedCmd = (speed_mes - speedCons);
			pwmCmd = PWM_RANGE*ROUND(speedCmd/mPerS2IncPerT(MAX_SPEED));
			if(pwmCmd >= PWM_RANGE)
				pwmCmd = PWM_RANGE;
			else if(pwmCmd <= 0)
				pwmCmd = 0;
#endif

			// Control motor
#ifdef ENCODER
			controlMotor(pwmCmd, dirCons, Drive);
#else
//			controlMotor(c*PWM_RANGE/7, dirCons, Drive);
			controlMotor(600, dirCons, Drive);
#endif
		}

	} // ############## End loop ############################################
}
