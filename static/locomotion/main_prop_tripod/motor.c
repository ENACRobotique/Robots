#include "motor.h"
#include "param.h"
#include "lpc214x.h"
#include "debug.h"
#include <pwm.h>
#include <sys_time.h> // for micros()

void controlMotor(int pwmCmd, eMotorDir dir, eMotorOperation motOp) {
    static eMotorOperation prvMotOp = FreeWheel;
    static unsigned startUs = 0;
    static eStateBstr prevStBstr = DisChgBstr;

    // Charge bootstrap capacitor
    if ((micros() - startUs) > PERIOD_CAPA1) {
        if (prevStBstr == DisChgBstr) {
            CHG_CAPA1_ON;
            prevStBstr = ChgBstr;
        }
        else if (prevStBstr == ChgBstr) {
            CHG_CAPA1_OFF;
            prevStBstr = DisChgBstr;
        }
        startUs = micros();
    }

    if (motOp != prvMotOp) {
        switch (prvMotOp) {
        case FreeWheel: // Reactivate half bridges drivers
            SD1_ON;
            SD2_ON;
            break;
        default:
            break;
        }
    }

    // Set command motor
    switch (motOp) {
    case FreeWheel: // "Open" all MOSFETs on the bridge
        SD1_OFF;
        SD2_OFF;
        break;
    case Drive: // Switching of only the low MOSFET (Q_LO1 or Q_LO2)
        if (dir == Trigo) {
            pwm_enable(1, pwmCmd);

            // set p0.7 to gpio; workaround to switch back the pin to gpio
            PCB_PINSEL0 &= ~(3 << 14);
            PCB_PINSEL0 |= 0 << 14;
            gpio_output(BK_PWM2, PIN_PWM2);
            gpio_write(BK_PWM2, PIN_PWM2, 0);
        }
        else if (dir == Notrigo) {
            pwm_enable(2, pwmCmd);

            // set p0.0 to gpio; workaround to switch back the pin to gpio
            PCB_PINSEL0 &= ~(3 << 0);
            PCB_PINSEL0 |= 0 << 0;
            gpio_output(BK_PWM1, PIN_PMW1);
            gpio_write(BK_PWM1, PIN_PMW1, 0);
        }
        else
            DEBUG_5_ON;
        break;
    case Braking: // Loop between motor & GND
        pwm_enable(1, PWM_RANGE);
        pwm_enable(2, PWM_RANGE);
        break;
    default:
        motOp = FreeWheel; // OMFG stay in a known state...
        break;
    }

    prvMotOp = motOp;
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
