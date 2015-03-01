/*
 * trajectory_manager.c
 *
 *  Created on: 22 févr. 2015
 *      Author: ludo6431, Yoyo
 *  Description:
 *  The trajectory manager allow to process some type of messages like
 *      a new speed set point
 *      a new elements of trajectory
 *      a new current position of the robot
 *  These messages are convert or adapted in units robots before to be stock or take into account
 *
 */

#include <string.h>
#include "params.h"
#include "trajectory_manager.h"

enum {
    S_WAIT, // no action asked (we are stopped)
    S_CHG_TRAJ, // new trajectory to follow
    S_RUN_TRAJ // we are following a trajectory
} state = S_WAIT; // state of the trajectory follow


//// Variables about the state of the robot
// Current position
int x, y; // Robot position on the table I << SHIFT
int theta = 0; // Robot heading on the table I.rad << SHIFT
// Goal
int gx = 0, gy = 0; // I << SHIFT
int gtheta = 0; // I << SHIFT

////// Variables for trajectory controller
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
void trajmngr_new_speed_sp(float speed) {
    /* Description:
     * Get the new speed set point value in units robots
     */
    speed_sp = isDpS2IpP(speed);
}

int trajmngr_new_traj_el(sTrajElRaw_t *te) {
    /* Description:
     * If a valid trajectory element is received thus the information within the message are
     * stocked in one of the two array following the current or the next trajectory
     * Return a negative value if an error occurs and 0 if no error
     */
    int error = 0;

    // We received some step of the current trajectory that we are following
    if (curr_traj_insert_sid > 0 && te->tid == curr_tid) {
        // Enough size in the array to add more trajectory elements
        if (curr_traj_insert_sid < TRAJ_MAX_SIZE) {
            // Expected follow up
            if (te->sid == curr_traj_insert_sid) {
                memcpy(&traj[curr_traj][curr_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
                traj[curr_traj][curr_traj_insert_sid].is_ok = 0;
                curr_traj_insert_sid++;
                state = S_RUN_TRAJ; // Follow the trajectory
            }
            else if (te->sid < curr_traj_insert_sid) {
                // Step already received (no error, could be caused by duplication in the network)
            }
            else {
                error = -1; // TODO error: bad step => invalidate all trajectory and ask new one
            }
        }
        // Too much trajectory steps received
        else {
            error = -2; // TODO
        }
    }
    // We received some step of the next trajectory, but we didn't switch to those (next tid is valid) FIXME
    else if (next_traj_insert_sid > 0 && te->tid == next_tid) {
        // Enough size in the array to add more trajectory elements
        if (next_traj_insert_sid < TRAJ_MAX_SIZE) {
            memcpy(&traj[!curr_traj][next_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
            traj[!curr_traj][next_traj_insert_sid].is_ok = 0;
            next_traj_insert_sid++;

            state = S_CHG_TRAJ; // New trajectory to follow
        }
        else if (te->sid < next_traj_insert_sid) {
            // Step already received (no error, could be caused by duplication in the network)
        }
        else {
            error = -3; // TODO error: bad step => invalidate all trajectory and ask new one
        }

    }
    // We received the first step of a new trajectory
    else {
        if (te->sid == 0) {
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
    if (error) {
        // TODO
    }

    return error;
}

void trajmngr_new_pos(sPosPayload *pos) {
    /* Description:
     * Stock and convert the current position and heading in robot units received by "bn_received function"
     * If the robot is motionless, the goal of robot is actualized
     */
    if (pos->id == 0) { // Keep information for primary robot
        x = isD2I(pos->x); // (I << SHIFT)
        y = isD2I(pos->y); // (I << SHIFT)
        theta = isROUND(D2I(WDIAM)*pos->theta); // (I.rad << SHIFT)

        if (state == S_WAIT) {
            gx = x;
            gy = y;
            gtheta = theta;
        }
    }
}

