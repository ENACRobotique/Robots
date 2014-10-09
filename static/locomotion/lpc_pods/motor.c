#include "motor.h"
#include "param.h"
#include "lpc214x.h"
#include "debug.h"
#include <pwm.h>
#include <sys_time.h> // for micros()

void controlMotor(int pwmCmd, eMotorDir dir, eMotorOperation motOp){
	static eMotorOperation prvMotOp = FreeWheel;
	static unsigned StChgtUs = 0;
//	static eStateBstr stBstr = DisChgBstr;

	// Charge bootstrap capacitor
	if((micros() - StChgtUs) > PERIOD_DCHT_CAPA1){
		CHG_CAPA1_ON;
		StChgtUs = micros();
		DEBUG_5_OFF;
	}
	else{
		CHG_CAPA1_OFF;
		DEBUG_5_ON;
	}

	if(motOp != prvMotOp){
		switch(prvMotOp){
		case FreeWheel: // Reactivate half bridges drivers
			SD1_ON;
			SD2_ON;
			break;
		default:
			break;
		}
	}

	// Set command motor
	switch(motOp){
	case FreeWheel: // "Open" all MOSFETs on the bridge
		SD1_OFF;
		SD2_OFF;
		prvMotOp = FreeWheel;
		break;
	case Drive: // Switching of only the low MOSFET (Q_LO1 or Q_LO2)
		if(dir == Trigo){
#ifdef LOW_CONSUMPTION
			// Alternating phase of free wheel & driving
			// TODO
#else
			// Alternating phase of breaking & driving
			pwm_enable(1, pwmCmd);
			//pwm_enable(2, PWM_RANGE); // TODO: GPIO
			// set p0.7 to gpio; workaround to switch back the pin to gpio
			PCB_PINSEL0 &= ~(3 << 14);
			PCB_PINSEL0 |= 0 << 14;
			gpio_output(BK_PWM2, PIN_PWM2);
			gpio_write(BK_PWM2, PIN_PWM2, 0);
#endif
		}
		else if(dir == Notrigo){
#ifdef LOW_CONSUMPTION
			// Alternating phase of free wheel & driving
			// TODO
#else
			// Alternating phase of breaking & driving
			pwm_enable(2, pwmCmd);
			//pwm_enable(1, PWM_RANGE); // TODO: GPIO
			// set p0.0 to gpio; workaround to switch back the pin to gpio
			PCB_PINSEL0 &= ~(3 << 0);
			PCB_PINSEL0 |= 0 << 0;
			gpio_output(BK_PWM1, PIN_PMW1);
			gpio_write(BK_PWM1, PIN_PMW1, 0);
#endif
		}
		else
			DEBUG_5_ON;
		break;
	case Braking: // Loop between motor & GND
		pwm_enable(1, PWM_RANGE);
		pwm_enable(2, PWM_RANGE);
		break;
	default:
		break;
	}
}

/*
 * MORE DETAIL cf the god LUDO
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
