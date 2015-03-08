/*
 * trajectory_controller.h
 *
 *  Created on: 11 févr. 2015
 *      Authors: yoyo
 *               Ludovic Lacoste
 */

#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include <encoder.h>
#include <encoders.h>
#include <motor.h>
#include <motors.h>
#include <mt_mat.h>
#include <pid.h>
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

    // Last known status
    int x, y; // (in I << SHIFT)
    int theta; // (in R << (RAD_SHIFT + SHIFT))

    // PID
    PID_t pid_traj;
    PID_t pid_orien;

    // Hardware objects
    encoder_t encs[NB_PODS];
    motor_t mots[NB_PODS];

    // Control object for speed of a POD
    speed_controller_t spd_ctls[NB_PODS];
    int next_spd_cmds[NB_PODS];
} trajectory_controller_t;

void trajctlr_init(trajectory_controller_t* tc, const int32_t mat_rob2pods[NB_PODS][NB_SPDS]);
void trajctlr_begin_update(trajectory_controller_t* tc);
void trajctlr_end_update(trajectory_controller_t* tc, int x_sp, int y_sp, int theta_sp);
void trajctlr_reset(trajectory_controller_t* tc);

#endif /* TRAJECTORY_CONTROLLER_H_ */
