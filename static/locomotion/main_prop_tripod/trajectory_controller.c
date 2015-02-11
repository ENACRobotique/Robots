/*
 * controller_trajectory.c
 *
 *  Created on: 11 févr. 2015
 *      Author: yoyo
 */

#include "trajectory_controller.h"
#include "param.h"
#include "tools.h"
#include "matrix.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "speed_controller.h"

void trajctl_init(trajectory_controller_t* ctl, MAT* mat_base, encoder_t ** tab_enc, motor_t ** tab_m, speed_controller_t ** tab_sp_ctl, int x_init, int y_init, int kp, int kd, int ki, int I_max, unsigned char shift) {
    int i;

    ctl->mat_sp_POD2b = mat_base;
    for(i=0; i<3; i++){
        ctl->enc[i] = *tab_enc[i];
        ctl->m[i] = *tab_m[i];
        ctl->spd_ctls[i] = *tab_sp_ctl[i];
    }
    ctl->x = x_init;
    ctl->y = y_init;

    pid_init(&(ctl->sPID), kp, ki, kd, I_max, shift);

}

void trajctl_update(trajectory_controller_t* ctl /* ,trajectory_sp(t), orientation_sp(t) */){
    // 0. Update ctl from trajectory_sp(t), orientation_sp(t)
     // TODO

    // 1. gets process value from the three encoders (V1_pv, V2_pv, V3_pv)
       get_speeds_pv(ctl->spd_ctls, (encoder_t**)&(ctl->enc)); // XXX

    // 2. compute new position and orientation from V1_pv, V2_pv, V3_pv to "x_pv", "y_pv", "o_pv"
       get_new_pos(ctl);

    // 3. trajectory control (some pid... ; gives speed orientation set point: Vx_cmd, Vy_cmd)
       ctl->Vx_cmd = pid_update(&(ctl->sPID), ctl->Vx_sp, ctl->Vx_pv);
       ctl->Vy_cmd = pid_update(&(ctl->sPID), ctl->Vy_sp, ctl->Vy_pv);

    // 4. orientation control (some pid as well... ; gives angular speed Omega_cmd)


    // 5. calculate speed set points for each pod (given Vx, Vy and Omega ; calculates V1_cmd, V2_cmd, V3_cmd; involves matrix multiplication)

    // 6. call speed controller with V1_cmd, V2_cmd, V3_cmd

    // 7. update motor command with results from speed controllers
}

void get_new_pos_orien(trajectory_controller_t* ctl){
    VEC* Vpos_pv = VNULL;
    VEC* V_sp_PODs = VNULL;

    // Fill the vector of speed of PODs
    V_sp_PODs->ve[0] = ctl->spd_ctls->speeds_pv[0];
    V_sp_PODs->ve[1] = ctl->spd_ctls->speeds_pv[1];
    V_sp_PODs->ve[2] = ctl->spd_ctls->speeds_pv[2];

    // Compute the speed of the base Vpos_pv = (Vx_pv, Vy_pv, O_pv) in the reference of the base
    Vpos_pv = mv_mlt(ctl->mat_sp_POD2b, V_sp_PODs, Vpos_pv);

    // Stock values
    ctl->Vx_pv = iROUND(Vpos_pv->ve[0]);
    ctl->Vy_pv = iROUND(Vpos_pv->ve[1]);
    ctl->O_pv = iROUND(Vpos_pv->ve[2]);

    // Compute the new position
    ctl->x = ctl->x_prev + ctl->Vx_pv*PER_ASSER;
    ctl->y = ctl->y_prev + ctl->Vy_pv*PER_ASSER;
    //ctl->o = iROUND((double)(ctl->o_prev + ctl->O_pv)%(2*M_PI)); //FIXME
}
