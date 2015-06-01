/*
 * position_controller.c
 *
 *  Created on: 11 févr. 2015
 *      Authors: Ludovic Lacoste, yoyo
 */

#include <mt_mat.h>
#ifdef ARCH_X86_LINUX
#   include <mt_io.h>
#endif
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
    mt_m_init(&tc->M_spds_rob2tpods, NB_PODS, NB_SPDS, MAT_SHIFT);
    for (i = 0; i < NB_PODS; i++) {
        MT_M_AT(&tc->M_spds_rob2tpods, i, 0)=  MT_M_AT(&tc->M_spds_rob2pods, i, 1);
        MT_M_AT(&tc->M_spds_rob2tpods, i, 1)= -MT_M_AT(&tc->M_spds_rob2pods, i, 0);
        MT_M_AT(&tc->M_spds_rob2tpods, i, 2)=  MT_M_AT(&tc->M_spds_rob2pods, i, 2);
    }

#if NB_PODS == NB_SPDS
    mt_m_inv(&tc->M_spds_rob2pods, &tc->M_spds_pods2rob);
#else
#error "Case where NB_PODS != NB_SPDS is not yet implemented"
#endif

#ifdef ARCH_X86_LINUX
    printf("rob2pods:");
    mt_m_foutput(&tc->M_spds_rob2pods, stdout);
    printf("pods2rob:");
    mt_m_foutput(&tc->M_spds_pods2rob, stdout);
#endif

    // Position uncertainty matrix
    mt_m_init(&tc->M_uncert_pos, NB_SPDS, NB_SPDS, VAR_POS_SHIFT); // FIXME max variance on start
    MT_M_AT(&tc->M_uncert_pos, 0, 0) = iROUND(D2I(10)*D2I(10)*dVarPosSHIFT); // (I² << VAR_POS_SHIFT)
    MT_M_AT(&tc->M_uncert_pos, 1, 1) = iROUND(D2I(10)*D2I(10)*dVarPosSHIFT); // (I² << VAR_POS_SHIFT)
    MT_M_AT(&tc->M_uncert_pos, 2, 2) = iROUND(0.7*0.7*dRadVarPosSHIFT); // (rad² << (RAD_SHIFT + VAR_POS_SHIFT))

    // Init encoders, motors and speed controllers
    motors_init(tc->mots);
    encoders_init(tc->encs, tc->mots);
    for (i = 0; i < NB_PODS; i++) {
        spdctlr_init(&tc->spd_ctls[i], &tc->encs[i]);
    }

    // Init PID
#define dPSHIFT ((double)(1 << SHIFT_PID_POS))

//    pid_init(&tc->pid_xtraj, iROUND(0.125 * dPSHIFT), iROUND(0.977e-3 * dPSHIFT), iROUND(15.6e-3 * dPSHIFT), 50 << SHIFT_PID_POS, SHIFT_PID_POS);
//    pid_init(&tc->pid_ytraj, iROUND(0.125 * dPSHIFT), iROUND(0.977e-3 * dPSHIFT), iROUND(15.6e-3 * dPSHIFT), 50 << SHIFT_PID_POS, SHIFT_PID_POS);
//    pid_init(&tc->pid_orien, iROUND(0.125 * dPSHIFT), iROUND(0.977e-3 * dPSHIFT), iROUND(15.6e-3 * dPSHIFT), 50 << SHIFT_PID_POS, SHIFT_PID_POS);

    double mul = 0.4;
    int KP = iROUND(mul*dPSHIFT*    0.125);
    int KI = iROUND(mul*dPSHIFT*    0.977e-3);
    int KD = iROUND(mul*dPSHIFT*    0);

    pid_init(&tc->pid_xtraj, KP, KI, KD, 200 << SHIFT_PID_POS, SHIFT_PID_POS);
    pid_init(&tc->pid_ytraj, KP, KI, KD, 200 << SHIFT_PID_POS, SHIFT_PID_POS);
    pid_init(&tc->pid_orien, KP, KI, KD, 200 << SHIFT_PID_POS, SHIFT_PID_POS);

    mf_init(&tc->mf_xtraj, 10);
    mf_init(&tc->mf_ytraj, 10);
    mf_init(&tc->mf_orien, 10);
}

void _update_pos_spd(position_controller_t* tc, MT_VEC *spd_pv_rob);

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

    // Get the speeds process values transformed in the robot's reference frame
    // (V1_pv, V2_pv, V3_pv) => (Vx_pv, Vy_pv, Oz_pv)
    mt_mv_mlt(&tc->M_spds_pods2rob, &spd_pv_pods, &spd_pv_rob);

    // Compute new position and orientation from
    // (Vx_pv, Vy_pv, Oz_pv, x, y, o) => (x, y, o)
    _update_pos_spd(tc, &spd_pv_rob);
}

void _linear_control(position_controller_t* tc, int x_sp, int y_sp, int vx_sp, int vy_sp, MT_VEC* spd_cmd_rob);
void _angular_control(position_controller_t* tc, int theta_sp, int oz_sp, MT_VEC* spd_cmd_rob);
void _update_pos_uncertainty(position_controller_t* tc, MT_VEC* spd_cmd_pods, MT_VEC* spd_cmd_tpods);

void posctlr_end_update(position_controller_t* tc, int x_sp, int y_sp, int theta_sp, int vx_sp, int vy_sp, int oz_sp) {
    int i;
    MT_VEC spd_cmd_rob = MT_V_INITS(NB_SPDS, VEC_SHIFT); // (Vx_cmd, Vy_cmd, Oz_pv) (in [IpP<<SHIFT]x[IpP<<SHIFT]x[radpP<<(RAD_SHIFT+SHIFT)])
    MT_VEC spd_cmd_pods = MT_V_INITS(NB_PODS, VEC_SHIFT); // (V1_cmd, V2_cmd, V3_cmd) (in IpP << SHIFT)

    //(some pid... ; gives linear speed set point: Vx_cmd, Vy_cmd)
    _linear_control(tc, x_sp, y_sp, vx_sp, vy_sp, &spd_cmd_rob);

    // (some pid as well... ; gives angular speed Oz_cmd)
    _angular_control(tc, theta_sp, oz_sp, &spd_cmd_rob);

    // Calculate speed set points for each pod
    // (Vx_cmd, Vy_cmd and Oz_cmd) => (V1_cmd, V2_cmd, V3_cmd)
    mt_mv_mlt(&tc->M_spds_rob2pods, &spd_cmd_rob, &spd_cmd_pods);

    // call speed controller with V1_cmd, V2_cmd, V3_cmd (in IpP<<SHIFT)
    for (i = 0; i < NB_PODS; i++) {
        spdctlr_update(&tc->spd_ctls[i], spd_cmd_pods.ve[i] >> SHIFT);
        tc->next_spd_cmds[i] = spdctlr_get(&tc->spd_ctls[i]);
    }

    // final step, update position uncertainty
    MT_VEC spd_cmd_tpods = MT_V_INITS(NB_PODS, VEC_SHIFT);
    mt_mv_mlt(&tc->M_spds_rob2tpods, &spd_cmd_rob, &spd_cmd_tpods);

    _update_pos_uncertainty(tc, &spd_cmd_pods, &spd_cmd_tpods);
}

void posctlr_set_pos(position_controller_t* tc, int x, int y, int theta) {
    tc->x = x;
    tc->y = y;
    tc->theta = theta;
    tc->vx = 0;
    tc->vy = 0;
    tc->oz = 0;
}

void posctrl_get_pos(position_controller_t* tc, int *x, int *y, int *theta) {
    *x = tc->x;
    *y = tc->y;
    *theta = tc->theta;
}

void posctrl_get_spd(position_controller_t* tc, int *vx, int *vy, int *oz) {
    *vx = tc->vx;
    *vy = tc->vy;
    *oz = tc->oz;
}

void posctlr_reset(position_controller_t* tc) {
    encoders_reset(tc->encs);
}

void _update_pos_spd(position_controller_t* tc, MT_VEC* spd_pv_rob) {
    // spd_pv_rob->ve[0..1] expressed in IpP << SHIFT
    // spd_pv_rob->ve[2] expressed in RpP << (RAD_SHIFT + SHIFT)

    // Update orientation and angular speed in playground reference frame
    tc->theta += spd_pv_rob->ve[2];
    tc->oz = spd_pv_rob->ve[2];

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

    // Update position and speed in playground reference frame
    tc->x += vx_pg;
    tc->y += vy_pg;

    tc->vx = vx_pg;
    tc->vy = vy_pg;
}

void _linear_control(position_controller_t* tc, int x_sp, int y_sp, int vx_sp, int vy_sp, MT_VEC* spd_cmd_rob) {
    // Compute the speeds command Vx_cmd, Vy_cmd (in playground reference frame) with pre-added estimated speed (feed forward)
    mf_update(&tc->mf_xtraj, pid_update(&tc->pid_xtraj, x_sp >> SHIFT, tc->x >> SHIFT) << SHIFT);
    mf_update(&tc->mf_ytraj, pid_update(&tc->pid_ytraj, y_sp >> SHIFT, tc->y >> SHIFT) << SHIFT);
    int vx_pg = vx_sp + mf_get(&tc->mf_xtraj);
    int vy_pg = vy_sp + mf_get(&tc->mf_ytraj);

    // and in robot reference frame
    spd_cmd_rob->ve[0] = (int32_t)(( (int64_t)tc->cos_theta * (int64_t)vx_pg + (int64_t)tc->sin_theta * (int64_t)vy_pg) >> SHIFT);
    spd_cmd_rob->ve[1] = (int32_t)((-(int64_t)tc->sin_theta * (int64_t)vx_pg + (int64_t)tc->cos_theta * (int64_t)vy_pg) >> SHIFT);
}

void _angular_control(position_controller_t* tc, int theta_sp, int oz_sp, MT_VEC* spd_cmd_rob) {
    while(theta_sp - tc->theta > issPI) {
        theta_sp -= (issPI << 1);
    }

    while(theta_sp - tc->theta < -issPI) {
        theta_sp += (issPI << 1);
    }

    // Compute the angular speed command Omega_cmd with pre-added estimated speed (feed forward)
    mf_update(&tc->mf_orien, pid_update(&tc->pid_orien, theta_sp >> SHIFT, tc->theta >> SHIFT) << SHIFT);
    spd_cmd_rob->ve[2] = oz_sp + mf_get(&tc->mf_orien); // do not right shift inputs of RAD_SHIFT, keep resolution
}

void _update_pos_uncertainty(position_controller_t* tc, MT_VEC* spd_cmd_pods, MT_VEC* spd_cmd_tpods) {
    MT_VEC diag_var_spds_pods = MT_V_INITS(NB_PODS, MAT_SHIFT);

    // TODO add some error if the sign of the setpoint changed (backlash)
    int std_spd_pod0 = ABS(spd_cmd_pods->ve[0] << (MAT_SHIFT - SHIFT - 6)) + ABS(spd_cmd_tpods->ve[0] << (MAT_SHIFT - SHIFT - 4));
    int std_spd_pod1 = ABS(spd_cmd_pods->ve[1] << (MAT_SHIFT - SHIFT - 6)) + ABS(spd_cmd_tpods->ve[1] << (MAT_SHIFT - SHIFT - 4));
    int std_spd_pod2 = ABS(spd_cmd_pods->ve[2] << (MAT_SHIFT - SHIFT - 6)) + ABS(spd_cmd_tpods->ve[2] << (MAT_SHIFT - SHIFT - 4));

    diag_var_spds_pods.ve[0] = SQRis(std_spd_pod0, MAT_SHIFT) >> VAR_PODS_DATASHIFT;
    diag_var_spds_pods.ve[1] = SQRis(std_spd_pod1, MAT_SHIFT) >> VAR_PODS_DATASHIFT;
    diag_var_spds_pods.ve[2] = SQRis(std_spd_pod2, MAT_SHIFT) >> VAR_PODS_DATASHIFT;

    MT_MAT M_rob2pg = MT_M_INITS(NB_SPDS, NB_SPDS, MAT_SHIFT);
    memset(M_rob2pg.me, 0, sizeof(*M_rob2pg.me)*M_rob2pg.rows*M_rob2pg.cols);
    MT_M_AT(&M_rob2pg, 0, 0) =  tc->cos_theta << (MAT_SHIFT - SHIFT); // FIXME calculate the full cos_theta and sin_theta, we have a loss of precision here
    MT_M_AT(&M_rob2pg, 0, 1) = -tc->sin_theta << (MAT_SHIFT - SHIFT);
    MT_M_AT(&M_rob2pg, 1, 0) =  tc->sin_theta << (MAT_SHIFT - SHIFT);
    MT_M_AT(&M_rob2pg, 1, 1) =  tc->cos_theta << (MAT_SHIFT - SHIFT);
    MT_M_AT(&M_rob2pg, 2, 2) =  1 << MAT_SHIFT;

    MT_MAT M_pods2pg = MT_M_INITS(NB_SPDS, NB_PODS, MAT_SHIFT);
    mt_mm_mlt(&M_rob2pg, &tc->M_spds_pods2rob, &M_pods2pg);

    MT_MAT M_uncert_spds = MT_M_INITS(NB_SPDS, NB_SPDS, MAT_SHIFT);
    mt_mtdm_mlt(&M_pods2pg, &diag_var_spds_pods, &M_uncert_spds);

    mt_m_mltshift(&M_uncert_spds, tc->M_uncert_pos.shift, VAR_PODS_DATASHIFT, &M_uncert_spds); // correct shift to be able to add value and shift data left by VAR_PODS_SHIFT
    mt_mm_add(&tc->M_uncert_pos, &M_uncert_spds, &tc->M_uncert_pos);

    // TODO clamp to max var

#ifdef ARCH_X86_LINUX
    printf("spd_cmd_pods:");
    mt_v_foutput(spd_cmd_pods, stdout);

    printf("diag_var_spds_pods:");
    mt_v_foutput(&diag_var_spds_pods, stdout);

    printf("M_pods2pg:");
    mt_m_foutput(&M_pods2pg, stdout);

    printf("M_uncert_spds:");
    mt_m_foutput(&M_uncert_spds, stdout);

    printf("M_uncert_pos:");
    mt_m_foutput(&tc->M_uncert_pos, stdout);
#endif
}
