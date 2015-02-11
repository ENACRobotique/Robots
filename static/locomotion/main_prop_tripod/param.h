#ifndef PARAM_H
#define PARAM_H

#include <gpio.h>

////// Pins LPC
//// Pins for POD1
// IN1
#define PIN_DIR_POD1 4
#define BK_DIR_POD1  0

// PWM1
#define PIN_PMW_POD1 7
#define BK_PWM_POD1  0
// Channel A
#define PIN_CHA_POD1 15
#define BK_CHA_POD1  0
// Channel B
#define PIN_CHB_POD1 14
#define BK_CHB_POD1  0

//// Pins for POD2
// DIR
#define PIN_DIR_POD2 5
#define BK_DIR_POD2  0
// PWM1
#define PIN_PMW_POD2 8
#define BK_PWM_POD2  0
// Channel A
#define PIN_CHA_POD2 17
#define BK_CHA_POD2  0
// Channel B
#define PIN_CHB_POD2 16
#define BK_CHB_POD2  0

//// Pins for POD3
// DIR
#define PIN_DIR_POD3 6
#define BK_DIR_POD3  0
// PWM1
#define PIN_PMW_POD3 9
#define BK_PWM_POD3  0
// Channel A
#define PIN_CHA_POD3 21
#define BK_CHA_POD3  0
// Channel B
#define PIN_CHB_POD3 20
#define BK_CHB_POD3  0

//// Pins for I2C
// Clock
#define PIN_SCL 2
#define BK_SCL  0
// Data
#define PIN_SDA 3
#define BK_SDA  0

//// Pins for UART
// TX
#define PIN_TX 0
#define BK_TX  0
// RX
#define PIN_RX 1
#define BK_RX  0

//// Pins for small switch
#define PIN_SWTCH1 28
#define BK_SWTCH1  0
#define PIN_SWTCH2 29
#define BK_SWTCH2  0
#define PIN_SWTCH3 30
#define BK_SWTCH3  0

#define PWM_RANGE 1024  // Don't change this value without change parameter of PWM

//// Parameters motor-reductor
#define REDUCT (676./49.)
#define SPIN2INC (1./REDUCT)
#define RES_ENC 500 // ticks/turn/channel

//// Parameters for asservissement
#define PER_SPEED_ASSER 20 // in ms
#define FRQ_ASSER (1./0.02)
#define MAX_SPEED 3 // in m/s

//// Wheel parameters
#ifdef SMALL_WHEEL
#define RAY_WHEEL  0.035 // in meters
#else
#define RAY_WHEEL (3.25*INCH2METERS)
#endif

#endif // PARAM_H
