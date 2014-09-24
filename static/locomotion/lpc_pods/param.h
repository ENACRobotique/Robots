#ifndef PARAM_H
#define PARAM_H

#include <gpio.h>

// Tests or methods of operation
	#define DVLPT_BOARD
//	#define ENCODER

////// Pins LPC
	//// Pins for motor control
		// IN1
		#define PIN_IN1 29
		#define BK_IN1  0
		// IN2
		#define PIN_IN2 28
		#define BK_IN2  0
		// PWM1
		#define PIN_PMW1 0
		#define BK_PWM1  0
		// PWM2
		#define PIN_PWM2 7
		#define BK_PWM2  0
	//// Pins for encoder
		// Channel A
		#define PIN_CNL_A 1
		#define BK_CNL_A  0
		// Channel B
		#define PIN_CNL_B 20
		#define BK_CNL_B  0
	//// Pins for I2C
		// Clock
		#define PIN_SCL 2
		#define BK_SCL  0
		// Data
		#define PIN_DATA 3
		#define BK_DATA  0
	//// Pins for UART
		// TX
		#define PIN_TX 8
		#define BK_TX  0
		// RX
		#define PIN_RX 9
		#define BK_RX  0
	//// Pins for small switch
		#define PIN_SWTCH1 13
		#define BK_SWTCH1  0
		#define PIN_SWTCH2 15
		#define BK_SWTCH2  0
		#define PIN_SWTCH3 17
		#define BK_SWTCH3  0
		#define PIN_SWTCH4 19
		#define BK_SWTCH4  0
	//// Pins for bootstrap (Load capacitors to on transistor)
		#define PIN_BSTRP 18
		#define BK_BSTRP  0


// Definition of pins to determine the direction of motor
#define IN1_ON gpio_write(BK_IN1, PIN_IN1, 1)
#define IN1_OFF gpio_write(BK_IN1, PIN_IN1, 0)
#define IN2_ON gpio_write(BK_IN2, PIN_IN2, 1)
#define IN2_OFF gpio_write(BK_IN2, PIN_IN2, 0)

#define IN1_VALUE gpio_read(BK_IN1, PIN_IN1)
#define IN2_VALUE gpio_read(BK_IN2, PIN_IN2)

#define PWM1_VALUE gpio_read(BK_PWM1, PIN_PWM1)
#define PWM2_VALUE gpio_read(BK_PWM2, PIN_PWM2)

#define ChannelA gpio_read(BK_CNL_A, PIN_CNL_A)
#define ChannelB gpio_read(BK_CNL_B, PIN_CNL_B)

#define sensTrigo {IN1_ON; IN2_OFF; DEBUG_1_ON; DEBUG_2_OFF;}
#define sensHorai {IN1_OFF; IN2_ON; DEBUG_2_ON; DEBUG_1_OFF;}
#define brake { IN1_OFF; IN2_OFF;}

#ifdef DVLPT_BOARD
#define READ_DIR_ASKED gpio_read(BK_SWTCH1, PIN_SWTCH1);
#else
// TODO
#endif

#define PWM_RANGE 1024  // Don't change this value without change parameter of PWM

//// Params for motor control
// Charge capacitor
#define PERIOD_DCHT_CAPA 8000 // period of motor control µs
#define PERIOD_CHGT_CAPA  10 // period of overload capacitor in us

//// Params motor-reductor
#define REDUCT (676./49.)
#define SPIN2INC (1/REDUCT)

//// Params for asservissement
#define T_ASSER 20 // in ms
#define FRQ_ASSER (1/0.02)
#define MAX_SPEED 3 // in m/s


//// Param wheel
#define RAY_WHEEL  0.035 // in meters


#endif // PARAM_H
