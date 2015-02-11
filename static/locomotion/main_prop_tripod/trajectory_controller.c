/*
 * controller_trajectory.c
 *
 *  Created on: 11 févr. 2015
 *      Author: yoyo
 */

#include "trajectory_controller.h"

void trajctl_init(trajectory_controller_t* ctl) {

}

void trajctl_update(trajectory_controller_t* ctl, /* trajectory_sp(t), orientation_sp(t) */){
    // 1. gets processvalue from the three encoders (V1_pv, V2_pv, V3_pv)

    // 2. compute new position and orientation from V1_pv, V2_pv, V3_pv to x_pv,y_pv,theta_pv

    // 3. trajectory control (some pid... ; gives speed orientation setpoint: Vx_cmd, Vy_cmd)

    // 4. orientation control (some pid as well... ; gives angular speed Omega_cmd)

    // 5. calculate speed setpoints for each pod (given Vx, Vy and Omega ; calculates V1_cmd, V2_cmd, V3_cmd; involves matrix multiplication)

    // 6. call speed controller with V1_cmd, V2_cmd, V3_cmd

    // 7. update motor command with results from speed controllers
}
