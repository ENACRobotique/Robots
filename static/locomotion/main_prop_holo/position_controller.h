/*
 * position_controller.h
 *
 *  Created on: 11 févr. 2015
 *      Authors: yoyo
 *               Ludovic Lacoste
 */

#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_

#include <encoder.h>
#include <encoders.h>
#include <motor.h>
#include <motors.h>
#include <mt_mat.h>
#include <pid.h>
#include <median_filter.h>
#include <speed_controller.h>
#include <stdint.h>

#if NB_ENCODERS != NB_MOTORS
#error "trajectory_controller's implementation assumes NB_ENCODERS==NB_MOTORS"
#endif

/**
 * Number of pods
 */
#define NB_PODS (NB_MOTORS)

/**
 * Number of internal speed outputs, must be 3 for a planar motion case (Vx, Vy, Omegaz)
 * current implementation assumes a value of 3, others will produce inconsistent results or buffer overflow
 */
#define NB_SPDS (3)

typedef struct {
    MT_MAT M_spds_pods2rob;
    MT_MAT M_spds_rob2pods;

    MT_MAT M_spds_rob2tpods; // transverse pods

    // position control status
    enum {
        PC_STATE_IDLE, // no control loop on position
        PC_STATE_RUNNING // control loop activated
    } state; // state of the trajectory follow

    // Last known status
    int x, y; // (in I << SHIFT)
    int theta; // (in R << (RAD_SHIFT + SHIFT))
    int cos_theta, sin_theta; // (in <<SHIFT)
    int vx, vy, oz; // (in IpP << SHIFT  or  RpP << (RAD_SHIFT + SHIFT))

    // Position uncertainty
    MT_MAT M_uncert_pos;

    // PID
    PID_t pid_xtraj;
    PID_t pid_ytraj;
    PID_t pid_orien;

    // median filter associated to PID
    median_t mf_xtraj;
    median_t mf_ytraj;
    median_t mf_orien;

    // Hardware objects
    encoder_t encs[NB_PODS];
    motor_t mots[NB_PODS];

    // Control object for speed of a POD
    speed_controller_t spd_ctls[NB_PODS];
    int next_spd_cmds[NB_PODS]; // in IpP
} position_controller_t;

void posctlr_init(position_controller_t* tc, const int32_t mat_rob2pods[NB_PODS][NB_SPDS]);
void posctlr_begin_update(position_controller_t* tc);
void posctlr_end_update(position_controller_t* tc, int x_sp, int y_sp, int theta_sp, int vx_sp, int vy_sp, int oz_sp);
void posctlr_set_pos(position_controller_t* tc, int x, int y, int theta);
void posctlr_get_pos(position_controller_t* tc, int *x, int *y, int *theta);
void posctlr_set_pos_u(position_controller_t* tc, int x_var, int y_var, int xy_var, int theta_var);
void posctlr_get_pos_u(position_controller_t* tc, int *x_var, int *y_var, int *xy_var, int *theta_var);
void posctlr_get_spd(position_controller_t* tc, int *vx, int *vy, int *oz);
void posctlr_reset(position_controller_t* tc);

void posctlr_stop(position_controller_t* tc);
void posctlr_run(position_controller_t* tc);

#endif /* POSITION_CONTROLLER_H_ */
