/*
 * trajectory_controller.h
 *
 *  Created on: 11 févr. 2015
 *      Author: yoyo
 */

#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include <matrix.h>

typedef struct{
    MAT *mat_sp_POD2b;

    //// data for trajectory control
    // Previous position
    int x_prev, y_prev;
    // Current position
    int x, y;
    int Vx_pv, Vy_pv;

    //// data for orientation control
    // Previous orientation
    int o_prev;
    // Current orientation
    int o;
    int O_pv;

    // Hardware objects
    encoder_t enc[3];
    motor_t m[3];

    // Control object for speed of a POD
    speed_controller_t spd_ctls[3];
} trajectory_controller_t;

void trajctl_init(trajectory_controller_t* ctl, MAT mat_base, encoder_t ** tab_enc, motor_t tab_m, speed_controller_t tab_sp_ctl, int x_init, int y_init);
void get_new_pos(trajectory_controller_t* ctl);

#endif /* TRAJECTORY_CONTROLLER_H_ */
