/*
 * trajectory_controller.h
 *
 *  Created on: 11 févr. 2015
 *      Author: yoyo
 */

#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include "matrix.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "speed_controller.h"

typedef struct{
    MAT *mat_sp_POD2b;
    MAT *mat_sp_b2POD;

    //// data for trajectory control
    // Previous position
    int x_prev, y_prev;
    // Current value (process values)
    int x, y;
    int Vx_pv, Vy_pv;

    //// data for orientation control
    // Previous orientation
    int o_prev;
    // Current value (process values)
    int o;
    int o_pv;

    // Set point values
    int Vx_sp, Vy_sp;
    int x_sp, y_sp;
    int o_sp;

    // Command values witch take account of trajectory and rotation
    int Vx_cmd, Vy_cmd, Omega_cmd;

    // PID
    PID_t sPID;

    // Hardware objects
    encoder_t enc[3];
    motor_t m[3];

    // Control object for speed of a POD
    speed_controller_t spd_ctls[3];
} trajectory_controller_t;

void trajctl_init(trajectory_controller_t* ctl, MAT* mat_base, encoder_t ** tab_enc, motor_t ** tab_m, speed_controller_t ** tab_sp_ctl, int x_init, int y_init, int kp, int kd, int ki, int I_max, unsigned char shift);
void get_new_pos_orien(trajectory_controller_t* ctl);
void trajectory_control(trajectory_controller_t* ctl);
void orientation_control(trajectory_controller_t* ctl);

#endif /* TRAJECTORY_CONTROLLER_H_ */
