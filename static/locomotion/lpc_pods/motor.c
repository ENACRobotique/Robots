#include "motor.h"
#include "param.h"
#include "lpc214x.h"
#include "debug.h"
#include <pwm.h>
#include <sys_time.h> // for micros()

void controlMotor(int pwmCmd, int dir, eMotorOperation motOp){
	static eMotorOperation prvMotOp = FreeWheel;
	static int prev_pwmValue = -1; // Must be negative for the initialization
	static int stateCapa = 0;
	static unsigned int timeUs;
	static unsigned prevChgtUs = 0;

	// Loading capacitor to turning on MOSFET 1 or 3
	timeUs = micros();
	switch (stateCapa) {
	case 0:
		if(timeUs - prevChgtUs > PERIOD_DCHT_CAPA){
			prevChgtUs = timeUs;
			gpio_write(BK_BSTRP, PIN_BSTRP, 0); // start loading , CHGT=12V
			stateCapa = 1; // loading...
		}
		break;
	case 1:
		if(timeUs - prevChgtUs > PERIOD_CHGT_CAPA){
			prevChgtUs = timeUs;
			gpio_write(BK_BSTRP, PIN_BSTRP, 1); // end loading, CHGT=0V
			stateCapa = 0;
		}
		break;
	}

	// Set command motor
	switch(motOp){
	case FreeWheel:
		prvMotOp = motOp;
		pwm_enable(1, 0);
		pwm_enable(2, 0);
		break;
	case Drive: // Clockwise rotation
		if(dir == 1){
			sensHorai;
		}
		else{
			sensTrigo;
		}
		if(pwmCmd != prev_pwmValue || prvMotOp != motOp){
			prvMotOp = motOp;
			pwm_enable(1, pwmCmd);
			pwm_enable(2, pwmCmd);
			prev_pwmValue = pwmCmd;
		}
		break;
	case Braking:
		if(pwmCmd != prev_pwmValue || prvMotOp != motOp){
			prvMotOp = motOp;
			brake;
			pwm_enable(1, pwmCmd);
			pwm_enable(2, pwmCmd);
			prev_pwmValue = pwmCmd;
		}
		break;
	default: ;
	}
}

/*
// set p0.0 to gpio; workaround to switch back the pin to gpio
					PCB_PINSEL0 &= ~(3 << 0);
					PCB_PINSEL0 |= 0 << 0;
					gpio_output(BK_PWM1, PIN_PMW1);
					gpio_write(BK_PWM1, PIN_PMW1, 1);

					// set p0.7 to gpio; workaround to switch back the pin to gpio
					PCB_PINSEL0 &= ~(3 << 14);
					PCB_PINSEL0 |= 0 << 14;
					gpio_output(BK_PWM2, PIN_PWM2);
					gpio_write(BK_PWM2, PIN_PWM2, 0);
*/
