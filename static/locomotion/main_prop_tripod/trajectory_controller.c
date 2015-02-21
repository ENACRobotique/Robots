/*
 * controller_trajectory.c
 *
 *  Created on: 11 févr. 2015
 *      Authors: Ludovic Lacoste, yoyo
 */

#include <mt_mat.h>
#include <params.h>
#include <pins.h>
#include <stdlib.h>
#include <string.h>
#include <tools.h>
#include <trajectory_controller.h>

#if NB_PODS != 3
#error "You can't change NB_PODS without changing trajectory_controller.c as well!"
#endif


typedef struct {
    union {
        struct { // data converted to fixed point (and in increments or in increments per period)
            // segment
            int p1_x;
            int p1_y;
            int p2_x;
            int p2_y;
            int seg_len;  // length of the trajectory on the segment
            // circle
            int c_x;
            int c_y;
            int c_r;
            int arc_len;  // length of the trajectory on the circle
            int arc_sp_max; // max_speed on the circle (radius dependent)
        };
        sTrajElRaw_t raw;
    };
    uint8_t is_ok;
} sTrajEl_t;

enum {
    S_WAIT, // no action asked (we are stopped)
    S_CHG_TRAJ, // new trajectory to follow
    S_RUN_TRAJ // we are following a trajectory
} state = S_WAIT; // state of the trajectory follow

//// Variables about the state of the robot
// Current position
int x_sp, y_sp; // Robot position on the table I << SHIFT
int theta_sp = 0; // Robot heading on the table I.rad << SHIFT
// Goal
int gx = 0, gy = 0; // I << SHIFT
int gtheta = 0; // I << SHIFT


//// Variables for trajectory controller
    // Set points
    int speed_sp = isDpS2IpP(SPEED_NOMI); // Desired speed (IpP<<SHIFT)
    // Trajectories
    volatile int curr_traj = 0; //  Trajectory witch is following 0 or 1
    int curr_traj_step; // Current step of the current trajectory [0:curr_traj_insert_sid*2-1}
    volatile int curr_traj_insert_sid = 0; // Index at which new trajectory steps must be added in the current trajectory
    volatile int next_traj_insert_sid = 0; // Index at which new trajectory steps will be add in the next trajectory
    volatile uint16_t curr_tid, next_tid; // Current and next trajectory identifiers
    sTrajEl_t traj[2][TRAJ_MAX_SIZE]; // Array to shock steps for two trajectory

//// Functions to update information after a received message
void new_speed_sp(float speed){
    /* Description:
     * Get the new speed set point value
     */
    speed_sp = isDpS2IpP(speed);
}

int new_traj_el(sTrajElRaw_t *te){
    /* Description:
     * If a valid trajectory element is received thus the information within the message are
     * stocked in one of the two array following the current or the next trajectory
     * Return a negative value if an error occurs and 0 if no error
     */
    int error = 0;

    // We received some step of the current trajectory that we are following
    if( curr_traj_insert_sid > 0 && te->tid == curr_tid ){
        // Enough size in the array to add more trajectory elements
        if( curr_traj_insert_sid < TRAJ_MAX_SIZE){
            // Expected follow up
            if( te->sid == curr_traj_insert_sid ){
                memcpy(&traj[curr_traj][curr_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
                traj[curr_traj][curr_traj_insert_sid].is_ok = 0;
                curr_traj_insert_sid++;
                state = S_RUN_TRAJ; // Follow the trajectory
            }
            else if( te->sid < curr_traj_insert_sid){
                // Step already received (no error, could be caused by duplication in the network)
            }
            else{
                error = -1; // TODO error: bad step => invalidate all trajectory and ask new one
            }
        }
        // Too much trajectory steps received
        else {
            error = -2; // TODO
        }
    }
    // We received some step of the next trajectory, but we didn't switch to those (next tid is valid) FIXME
    else if( next_traj_insert_sid > 0 && te->tid == next_tid){
        // Enough size in the array to add more trajectory elements
        if( next_traj_insert_sid < TRAJ_MAX_SIZE) {
            memcpy(&traj[!curr_traj][next_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
            traj[!curr_traj][next_traj_insert_sid].is_ok = 0;
            next_traj_insert_sid++;

            state = S_CHG_TRAJ; // New trajectory to follow
        }
        else if( te->sid < next_traj_insert_sid){
            // Step already received (no error, could be caused by duplication in the network)
        }
        else {
            error = -3; // TODO error: bad step => invalidate all trajectory and ask new one
        }

    }
    // We received the first step of a new trajectory
    else{
        if (te->sid == 0){
            next_traj_insert_sid = 0; // Reset the index for the next trajectory
            next_tid = te->tid; // get the next tid
            memcpy(&traj[!curr_traj][next_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
            traj[!curr_traj][next_traj_insert_sid].is_ok = 0;
            next_traj_insert_sid++;

            state = S_CHG_TRAJ;
        }
        else {
            error = -5; // TODO
        }
    }

    // Processing of the error
    if (error){
        // TODO
    }

    return error;
}

void new_pos(sPosPayload *pos){
    /* Description:
       * Stock and convert the position and heading in robot units
       * If the robot is motionless, the goal of robot is actualize
       */
    if(pos->id == 0){ // Keep information for primary robot
        x_sp = isD2I(pos->x); // (I << SHIFT)
        y_sp = isD2I(pos->y); // (I << SHIFT)
        theta_sp = isROUND(D2I(WDIAM)*pos->theta); // (I.rad << SHIFT)

        if(state == S_WAIT){
            gx = x_sp;
            gy = y_sp;
            gtheta = theta_sp;
        }
    }
}


void trajctl_init(trajectory_controller_t* ctl, const int32_t mat_rob2pods[NB_PODS][NB_SPDS]) {
    int i, j;
    memset(ctl, 0, sizeof(*ctl));

    // Transformation matrices
    mt_m_init(&ctl->M_spds_pods2rob, NB_SPDS, NB_PODS, MAT_SHIFT);
    mt_m_init(&ctl->M_spds_rob2pods, NB_PODS, NB_SPDS, MAT_SHIFT);
    for (i = 0; i < NB_PODS; i++) {
        for (j = 0; j < NB_SPDS; j++) {
            MT_M_AT(&ctl->M_spds_rob2pods, i, j) = mat_rob2pods[i][j];
        }
    }
#if NB_PODS == NB_SPDS
    mt_m_inv(&ctl->M_spds_rob2pods, &ctl->M_spds_pods2rob);
#else
#error "Case where NB_PODS != NB_SPDS is not yet implemented"
#endif

    // Init encoders, motors and speed controllers
    encoders_init(ctl->encs);
    motors_init(ctl->mots);
    for (i = 0; i < NB_PODS; i++) {
        spdctl_init(&ctl->spd_ctls[i], &ctl->encs[i]);
    }

    // Init PID
    pid_init(&ctl->pid_traj, 1, 0, 0, 0, 0); // TODO find good values
    pid_init(&ctl->pid_orien, 1, 0, 0, 0, 0); // TODO find good values
}

void _update_pos_orien(trajectory_controller_t* ctl, MT_VEC *spd_pv_rob);
void _trajectory_control(trajectory_controller_t* ctl, int x_sp, int y_sp, MT_VEC* spd_cmd_rob);
void _orientation_control(trajectory_controller_t* ctl, int o_sp, MT_VEC* spd_cmd_rob);

void trajctl_update(trajectory_controller_t* ctl /* ,trajectory_sp(t), orientation_sp(t) */) {
    int i;
    MT_VEC spd_pv_pods = MT_V_INITS(NB_PODS, VEC_SHIFT); // (V1_pv, V2_pv, V3_pv)
    MT_VEC spd_pv_rob = MT_V_INITS(NB_SPDS, VEC_SHIFT); // (Vx_pv, Vy_pv, Oz_pv)
    MT_VEC spd_cmd_rob = MT_V_INITS(NB_SPDS, VEC_SHIFT); // (Vx_cmd, Vy_cmd, Oz_pv)
    MT_VEC spd_cmd_pods = MT_V_INITS(NB_PODS, VEC_SHIFT); // (V1_cmd, V2_cmd, V3_cmd)


    // Gets speed process values from the three encoders
    // => V1_pv, V2_pv, V3_pv and backups the nbticks
    for (i = 0; i < NB_PODS; i++) {
        encoder_update(&ctl->encs[i]);
        spd_pv_pods.ve[i] = encoder_get(&ctl->encs[i]);
    }

    // Send previously computed command to the motors // FIXME why now, is it too late?
    for (i = 0; i < NB_PODS; i++) {
        motor_update(&ctl->mots[i], ctl->next_spd_cmds[i]);
    }

    // Get the speeds process values transformed in the robot's reference frame
    // (V1_pv, V2_pv, V3_pv) => (Vx_pv, Vy_pv, Oz_pv)
    mt_mv_mlt(&ctl->M_spds_pods2rob, &spd_pv_pods, &spd_pv_rob);

    // Compute new position and orientation from
    // (Vx_pv, Vy_pv, Oz_pv, x, y, o) => (x, y, o)
    _update_pos_orien(ctl, &spd_pv_rob);

    // Trajectory control
    //(some pid... ; gives speed orientation set point: Vx_cmd, Vy_cmd)
    _trajectory_control(ctl, x_sp, y_sp, &spd_cmd_rob);

    // Orientation control
    // (some pid as well... ; gives angular speed Oz_cmd)
    _orientation_control(ctl, theta_sp, &spd_cmd_rob);

    // Calculate speed set points for each pod
    // (Vx_cmd, Vy_cmd and Oz_cmd) => (V1_cmd, V2_cmd, V3_cmd)
    mt_mv_mlt(&ctl->M_spds_rob2pods, &spd_cmd_rob, &spd_cmd_pods);

    // call speed controller with V1_cmd, V2_cmd, V3_cmd
    for (i = 0; i < NB_PODS; i++) {
        spdctl_update(&ctl->spd_ctls[i], spd_cmd_pods.ve[i]);
        ctl->next_spd_cmds[i] = spdctl_get(&ctl->spd_ctls[i]);
    }
}

void _update_pos_orien(trajectory_controller_t* ctl, MT_VEC* spd_pv_rob) {
    // Compute the new position
    ctl->x += spd_pv_rob->ve[0];
    ctl->y += spd_pv_rob->ve[1];
    ctl->o += spd_pv_rob->ve[2]; // FIXME, make a modulo
}

void _trajectory_control(trajectory_controller_t* ctl, int x_sp, int y_sp, MT_VEC* spd_cmd_rob) {
//    int errx, erry;
//
//    // Limit acceleration
//    errx = x_sp - ctl->x;
//    erry = y_sp - ctl->y;
//
//    if (abs(errx) >= MAX_ACC) {
//        errx = SIGN(errx) * MAX_ACC;
//        x_sp = ctl->x + errx;
//    }
//
//    if (abs(erry) >= MAX_ACC) {
//        erry = SIGN(erry) * MAX_ACC;
//        y_sp = ctl->y + erry;
//    }

    // Compute the speeds command Vx_cmd, Vy_cmd
    spd_cmd_rob->ve[0] = pid_update(&ctl->pid_traj, x_sp, ctl->x);
    spd_cmd_rob->ve[1] = pid_update(&ctl->pid_traj, y_sp, ctl->y);
}

void _orientation_control(trajectory_controller_t* ctl, int o_sp, MT_VEC* spd_cmd_rob) {
//    int erro;
//
//    // Limit angular acceleration
//    erro = o_sp - ctl->o;
//
//    if (abs(erro) >= MAX_ANG_ACC) {
//        erro = SIGN(erro) * MAX_ANG_ACC;
//        o_sp = ctl->o + erro;
//    }

    // Compute the angular speed command Omega_cmd
    spd_cmd_rob->ve[2] = pid_update(&ctl->pid_orien, o_sp, ctl->o);
}
