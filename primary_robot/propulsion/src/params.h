/*
 * params.h
 *
 *  Created on: 6 mars 2017
 *      Author: fabien
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#define ODO_I_LEFT 2
#define ODO_S_LEFT 3
#define ODO_I_RIGHT 4
#define ODO_S_RIGHT 5

#define DIR_LEFT 6
#define PWM_LEFT 7
#define DIR_RIGHT 8
#define PWM_RIGHT 9

#define SERVO1 20
#define SERVO2 21
#define SERVO3 22
#define SERVO4 23
#define MOTOR_PICKER 29
#define MOTOR_LAUNCHER 30

#define BUTEE_LEFT 27
#define BUTEE_RIGHT 28
#define IHM_BONUS 35
#define LED_RED 36
#define LED_GREEN 37
#define LED_BLUE 38


#define SERVO_TRAP SERVO4
#define SERVO_SORTER SERVO3
#define SERVO_CUTTER SERVO2

#define LAUNCHER_SPEED 100
#define PICKER_SPEED 512

#define GO_TRAP_CLOSE 787 //In ms
#define GO_TRAP_OPEN 1222 //In ms
#define COLLECT_BALL_1 1350 //In ms
#define COLLECT_BALL_2 1610 //In ms
#define GO_SORTER_UP 2365 //In ms
#define GO_CUTTER_CLOSE 100 //In °
#define GO_CUTTER_OPEN 84 //In °

#define UPDATE_PERIOD 0.02f
#define REPORT_POSITION_TIMER 3000000

#define MM_TO_INC 28.64
#define RAD_TO_INC 7900.0 //6432.54
#define RAD_TO_UINT16 10430.378350470453

#define SPEED_COEFF 78
/*
 * All pwm on the same timer have the same frequency.
 * Timer | pwm pins
 *  FTM0 | 5, 6, 9, 10, 20, 21, 22, 23
 *  FTM1 | 3, 4
 *  FTM2 | 29, 30
 *  FTM3 | 2, 7, 8, 14, 35, 36, 37, 38
 */

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define RESTART() WRITE_RESTART(0x5FA0004);


#endif /* PARAMS_H_ */
