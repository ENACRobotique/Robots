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

    // Transition matrixes
    ctl->mat_sp_POD2b = mat_base;
    ctl->mat_sp_b2POD = m_inverse(mat_base, ctl->mat_sp_b2POD);

    // Init structures encoder, motor and speed controller
    for(i=0; i<3; i++){
        ctl->enc[i] = *tab_enc[i];
        ctl->m[i] = *tab_m[i];
        ctl->spd_ctls[i] = *tab_sp_ctl[i];
    }

    // Get the initial position
    ctl->x = x_init;
    ctl->y = y_init;

    pid_init(&(ctl->sPID), kp, ki, kd, I_max, shift);
}

void trajctl_update(trajectory_controller_t* ctl /* ,trajectory_sp(t), orientation_sp(t) */){
    // 0. Update ctl from trajectory_sp(t), orientation_sp(t)
     // TODO

    // 1. Gets process value from the three encoders
       // => V1_pv, V2_pv, V3_pv and duplicates the nbticks
       get_enc_pv(ctl->spd_ctls, (encoder_t**)&(ctl->enc)); // XXX

    // 2. Compute new position and orientation from
       // V1_pv, V2_pv, V3_pv => "x_pv", "y_pv", "o_pv"
       get_new_pos_orien(ctl);

    // 3. Trajectory control
       //(some pid... ; gives speed orientation set point: Vx_cmd, Vy_cmd)
       trajectory_control(ctl);

    // 4. Orientation control
       // (some pid as well... ; gives angular speed Omega_cmd)
       orientation_control(ctl);

    // 5. Calculate speed set points for each pod
       // Vx, Vy and Omega => V1_cmd, V2_cmd, V3_cmd
       speed_cmd_PODs(ctl);

    // 6. call speed controller with V1_cmd, V2_cmd, V3_cmd
       motors_control(ctl);

    // 7. update motor command with results from speed controllers
       motors_update(ctl);
}

void get_new_pos_orien(trajectory_controller_t* ctl){
    VEC* V_sp_b_pv = VNULL;
    VEC* V_sp_PODs = VNULL;

    // Fill the vector of speed of PODs
    V_sp_PODs->ve[0] = ctl->spd_ctls->speeds_pv[0];
    V_sp_PODs->ve[1] = ctl->spd_ctls->speeds_pv[1];
    V_sp_PODs->ve[2] = ctl->spd_ctls->speeds_pv[2];

    // Compute the speed of the base V_sp_b_pv = (Vx_pv, Vy_pv, O_pv) in the reference of the base
    V_sp_b_pv = mv_mlt(ctl->mat_sp_POD2b, V_sp_PODs, V_sp_b_pv);

    // Stock values
    ctl->Vx_pv = iROUND(V_sp_b_pv->ve[0]);
    ctl->Vy_pv = iROUND(V_sp_b_pv->ve[1]);
    ctl->o_pv = iROUND(V_sp_b_pv->ve[2]);

    // Compute the new position
    ctl->x = ctl->x_prev + ctl->Vx_pv*PER_ASSER;
    ctl->y = ctl->y_prev + ctl->Vy_pv*PER_ASSER;
    ctl->o = ctl->o_prev + ctl->o_pv; // FIXME, make a modulo
}

void trajectory_control(trajectory_controller_t* ctl){
    int errx, erry;

    // Limit acceleration (keeping ratio of errors constant)
    errx = ctl->x_sp - ctl->x;
    erry = ctl->y_sp - ctl->y;

    if(abs(errx)>= MAX_ACC){
        errx = SIGN(errx)*MAX_ACC;
        ctl->x_sp = ctl->x + errx;
    }

    if(abs(erry)>= MAX_ACC){
        erry = SIGN(erry)*MAX_ACC;
        ctl->y_sp = ctl->y + erry;
    }

    // Compute the speeds command Vx_cmd, Vy_cmd
    ctl->Vx_cmd = pid_update(ctl->sPID, ctl->x_sp, ctl->x);
    ctl->Vy_cmd = pid_update(ctl->sPID, ctl->y_sp, ctl->y);

}

void orientation_control(trajectory_controller_t* ctl){
    int erro;

    // Limit angular acceleration
    erro = ctl->o_sp - ctl->o_pv;

    if(abs(erro)>= MAX_ANG_ACC){
        erro = SIGN(erro)*MAX_ANG_ACC;
        ctl->o_sp = ctl->o + erro;
    }

    // Compute the angular speed command Omega_cmd
    ctl->Omega_cmd = pid_update(ctl->sPID, ctl->o_sp, ctl->o);
}

void speed_cmd_PODs(trajectory_controller_t* ctl){
    VEC* V_sp_cmd_b = VNULL;
    VEC* V_sp_PODs = VNULL;

    // Fill the vector of speed cmd in the base
    V_sp_cmd_b->ve[0] = ctl->Vx_cmd;
    V_sp_cmd_b->ve[1] = ctl->Vy_cmd;
    V_sp_cmd_b->ve[2] = ctl->Omega_cmd;

    // Compute the speeds for PODs
    V_sp_PODs = mv_mlt(ctl->mat_sp_b2POD, V_sp_cmd_b, V_sp_PODs);

    // Stock values
    ctl->spd_ctls->speeds_cmd[0] = V_sp_PODs->ve[0];
    ctl->spd_ctls->speeds_cmd[1] = V_sp_PODs->ve[1];
    ctl->spd_ctls->speeds_cmd[2] = V_sp_PODs->ve[2];
}

void motors_control(trajectory_controller_t* ctl){
    // TODO
}

void motors_update(trajectory_controller_t* ctl){
    // TODO
}
