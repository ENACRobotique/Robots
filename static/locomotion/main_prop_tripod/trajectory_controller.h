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

#if NB_ENCODERS != 3 || NB_MOTORS != 3
#error "You can't change NB_ENCODERS or NB_MOTORS without changing trajectory_controller as well!"
#endif

// do not use those defines from motors.h and encoders.h, use NB_PODS
#undef NB_ENCODERS
#undef NB_MOTORS

/**
 * Number of pods (must be equal to NB_MOTORS and NB_ENCODERS)
 */
#define NB_PODS (3)

/**
 * Number of internal speed outputs, must be 3 for a planar motion case (Vx, Vy, Omega)
 */
#define NB_SPDS (3)

typedef struct {
    MT_MAT M_spds_pods2rob;
    MT_MAT M_spds_rob2pods;

    // Last known status
    int x, y, theta;

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

void trajctlr_init(trajectory_controller_t* ctl, const int32_t mat_rob2pods[NB_PODS][NB_SPDS]);
void trajctlr_update(trajectory_controller_t* ctl /* ,trajectory_sp(t), orientation_sp(t) */);
void trajctlr_reset(trajectory_controller_t* ctl);

#endif /* TRAJECTORY_CONTROLLER_H_ */
