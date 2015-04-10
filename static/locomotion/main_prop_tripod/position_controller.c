/*
 * position_controller.c
 *
 *  Created on: 11 févr. 2015
 *      Authors: Ludovic Lacoste, yoyo
 */

#include <mt_mat.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "params.h"
#include "tools.h"

#include "position_controller.h"

#if SHIFT != VEC_SHIFT
#error "trajectory_controller's implementation assumes SHIFT==VEC_SHIFT"
#endif

void posctlr_init(position_controller_t* tc, const int32_t mat_rob2pods[NB_PODS][NB_SPDS]) {
    int i, j;
    memset(tc, 0, sizeof(*tc));

    // Transformation matrices
    mt_m_init(&tc->M_spds_pods2rob, NB_SPDS, NB_PODS, MAT_SHIFT);
    mt_m_init(&tc->M_spds_rob2pods, NB_PODS, NB_SPDS, MAT_SHIFT);
    for (i = 0; i < NB_PODS; i++) {
        for (j = 0; j < NB_SPDS; j++) {
            MT_M_AT(&tc->M_spds_rob2pods, i, j)= mat_rob2pods[i][j];
        }
    }
#if NB_PODS == NB_SPDS
    mt_m_inv(&tc->M_spds_rob2pods, &tc->M_spds_pods2rob);
#else
#error "Case where NB_PODS != NB_SPDS is not yet implemented"
#endif

    // Init encoders, motors and speed controllers
    motors_init(tc->mots);
    encoders_init(tc->encs, tc->mots);
    for (i = 0; i < NB_PODS; i++) {
        spdctlr_init(&tc->spd_ctls[i], &tc->encs[i]);
    }

    // Init PID
//    pid_init(&tc->pid_xtraj, 1 << (SHIFT_PID_TRAJ_POS - 3), 1 << (SHIFT_PID_TRAJ_POS - 8), 0, 50 << SHIFT_PID_TRAJ_POS, SHIFT_PID_TRAJ_POS); // TODO find good values
//    pid_init(&tc->pid_ytraj, 1 << (SHIFT_PID_TRAJ_POS - 3), 1 << (SHIFT_PID_TRAJ_POS - 8), 0, 50 << SHIFT_PID_TRAJ_POS, SHIFT_PID_TRAJ_POS); // TODO find good values

    pid_init(&tc->pid_xtraj, 1 << (SHIFT_PID_TRAJ_POS - 3), 1 << (SHIFT_PID_TRAJ_POS - 10), 1 << (SHIFT_PID_TRAJ_POS - 6), 50 << SHIFT_PID_TRAJ_POS, SHIFT_PID_TRAJ_POS); // TODO find good values
    pid_init(&tc->pid_ytraj, 1 << (SHIFT_PID_TRAJ_POS - 3), 1 << (SHIFT_PID_TRAJ_POS - 10), 1 << (SHIFT_PID_TRAJ_POS - 6), 50 << SHIFT_PID_TRAJ_POS, SHIFT_PID_TRAJ_POS); // TODO find good values
    pid_init(&tc->pid_orien, 1 << (SHIFT_PID_TRAJ_ANG - 4), 1 << (SHIFT_PID_TRAJ_ANG - 10), 1 << (SHIFT_PID_TRAJ_ANG - 6), 50 << SHIFT_PID_TRAJ_ANG, SHIFT_PID_TRAJ_ANG); // TODO find good values
}

void _update_pos_orien(position_controller_t* tc, MT_VEC *spd_pv_rob);

void posctlr_begin_update(position_controller_t* tc) {
    int i;
    MT_VEC spd_pv_pods = MT_V_INITS(NB_PODS, VEC_SHIFT); // (V1_pv, V2_pv, V3_pv) (in IpP<<SHIFT)
    MT_VEC spd_pv_rob = MT_V_INITS(NB_SPDS, VEC_SHIFT);// (Vx_pv, Vy_pv, Oz_pv) (in [IpP<<SHIFT]x[IpP<<SHIFT]x[radpP<<(RAD_SHIFT+SHIFT)])

    // Gets speed process values from the three encoders
    // => V1_pv, V2_pv, V3_pv and backups the nbticks
    for (i = 0; i < NB_PODS; i++) {
        encoder_update(&tc->encs[i]);
        spd_pv_pods.ve[i] = encoder_get(&tc->encs[i]) << SHIFT;
    }

    // Send previously computed command to the motors
    for (i = 0; i < NB_PODS; i++) {
        motor_update(&tc->mots[i], tc->next_spd_cmds[i]);
    }

//    printf("%i, %i, %i, ", tc->encs[0].nbticks_cache, tc->encs[1].nbticks_cache, tc->encs[2].nbticks_cache);

    // Get the speeds process values transformed in the robot's reference frame
    // (V1_pv, V2_pv, V3_pv) => (Vx_pv, Vy_pv, Oz_pv)
    mt_mv_mlt(&tc->M_spds_pods2rob, &spd_pv_pods, &spd_pv_rob);

    // Compute new position and orientation from
    // (Vx_pv, Vy_pv, Oz_pv, x, y, o) => (x, y, o)
    _update_pos_orien(tc, &spd_pv_rob);
}

void _trajectory_control(position_controller_t* tc, int x_sp, int y_sp, int vx_sp, int vy_sp, MT_VEC* spd_cmd_rob);
void _orientation_control(position_controller_t* tc, int theta_sp, int oz_sp, MT_VEC* spd_cmd_rob);

void posctlr_end_update(position_controller_t* tc, int x_sp, int y_sp, int theta_sp, int vx_sp, int vy_sp, int oz_sp) {
    int i;
    MT_VEC spd_cmd_rob = MT_V_INITS(NB_SPDS, VEC_SHIFT); // (Vx_cmd, Vy_cmd, Oz_pv) (in [IpP<<SHIFT]x[IpP<<SHIFT]x[radpP<<(RAD_SHIFT+SHIFT)])
    MT_VEC spd_cmd_pods = MT_V_INITS(NB_PODS, VEC_SHIFT); // (V1_cmd, V2_cmd, V3_cmd) (in IpP << SHIFT)

    // Trajectory control
    //(some pid... ; gives speed orientation set point: Vx_cmd, Vy_cmd)
    _trajectory_control(tc, x_sp, y_sp, vx_sp, vy_sp, &spd_cmd_rob);

    // Orientation control
    // (some pid as well... ; gives angular speed Oz_cmd)
    _orientation_control(tc, theta_sp, oz_sp, &spd_cmd_rob);

    // Calculate speed set points for each pod
    // (Vx_cmd, Vy_cmd and Oz_cmd) => (V1_cmd, V2_cmd, V3_cmd)
    mt_mv_mlt(&tc->M_spds_rob2pods, &spd_cmd_rob, &spd_cmd_pods);

//    printf("%i, %i, %i\n", spd_cmd_pods.ve[0], spd_cmd_pods.ve[1], spd_cmd_pods.ve[2]);

    // call speed controller with V1_cmd, V2_cmd, V3_cmd (in IpP<<SHIFT)
    for (i = 0; i < NB_PODS; i++) {
        spdctlr_update(&tc->spd_ctls[i], spd_cmd_pods.ve[i] >> SHIFT);
        tc->next_spd_cmds[i] = spdctlr_get(&tc->spd_ctls[i]);
    }
}

void posctlr_set_pos(position_controller_t* tc, int x, int y, int theta) {
    tc->x = x;
    tc->y = y;
    tc->theta = theta;
}

void posctlr_reset(position_controller_t* tc) {
    encoders_reset(tc->encs);
}

void _update_pos_orien(position_controller_t* tc, MT_VEC* spd_pv_rob) {
    // spd_pv_rob->ve[0..1] expressed in IpP << SHIFT
    // spd_pv_rob->ve[2] expressed in RpP << (RAD_SHIFT + SHIFT)

    // Update orientation in playground reference frame
    tc->theta += spd_pv_rob->ve[2];

    // get theta's principal angle value
    if(tc->theta > issPI) {
        tc->theta -= (issPI << 1); // 2PI
    }
    else if(tc->theta < -issPI) {
        tc->theta += (issPI << 1); // 2PI
    }
    SINCOS(tc->theta, &tc->sin_theta, &tc->cos_theta);

    // speed in robot reference frame (in IpP<<SHIFT)
    int vx_rob = spd_pv_rob->ve[0];
    int vy_rob = spd_pv_rob->ve[1];

    // speed in playground reference frame (in IpP<<SHIFT)
    int vx_pg = (int32_t)(((int64_t)tc->cos_theta * (int64_t)vx_rob - (int64_t)tc->sin_theta * (int64_t)vy_rob) >> SHIFT);
    int vy_pg = (int32_t)(((int64_t)tc->sin_theta * (int64_t)vx_rob + (int64_t)tc->cos_theta * (int64_t)vy_rob) >> SHIFT);

    // Update position in playground reference frame
    tc->x += vx_pg;
    tc->y += vy_pg;
}

void _trajectory_control(position_controller_t* tc, int x_sp, int y_sp, int vx_sp, int vy_sp, MT_VEC* spd_cmd_rob) {
    // Compute the speeds command Vx_cmd, Vy_cmd (in playground reference frame) with pre-added estimated speed (feed forward)
    int vx_pg = vx_sp + (pid_update(&tc->pid_xtraj, x_sp >> SHIFT, tc->x >> SHIFT) << SHIFT);
    int vy_pg = vy_sp + (pid_update(&tc->pid_ytraj, y_sp >> SHIFT, tc->y >> SHIFT) << SHIFT);

    // and in robot reference frame
    spd_cmd_rob->ve[0] = (int32_t)(( (int64_t)tc->cos_theta * (int64_t)vx_pg + (int64_t)tc->sin_theta * (int64_t)vy_pg) >> SHIFT);
    spd_cmd_rob->ve[1] = (int32_t)((-(int64_t)tc->sin_theta * (int64_t)vx_pg + (int64_t)tc->cos_theta * (int64_t)vy_pg) >> SHIFT);
}

void _orientation_control(position_controller_t* tc, int theta_sp, int oz_sp, MT_VEC* spd_cmd_rob) {
    while(theta_sp - tc->theta > issPI) {
        theta_sp -= (issPI << 1);
    }

    while(theta_sp - tc->theta < -issPI) {
        theta_sp += (issPI << 1);
    }

    // Compute the angular speed command Omega_cmd with pre-added estimated speed (feed forward)
    spd_cmd_rob->ve[2] = oz_sp + (pid_update(&tc->pid_orien, theta_sp >> SHIFT, tc->theta >> SHIFT) << SHIFT); // do not right shift inputs of RAD_SHIFT, keep resolution
}
