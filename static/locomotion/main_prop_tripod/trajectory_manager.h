/*
 * trajectory_manager.h
 *
 *  Created on: 22 févr. 2015
 *      Author: ludo6431
 */

#ifndef TRAJECTORY_MANAGER_H_
#define TRAJECTORY_MANAGER_H_

#include <messages-locomotion.h>
#include <messages-position.h>
#include <stdint.h>

#include "tools.h"

#define TRAJ_MAX_SLOTS (16)

// data converted to fixed point (and in increments or in increments per period)
typedef struct {
    // segment
    uint32_t seg_start_date;        // (in microseconds)
    int32_t seg_start_theta;        // (I.rad << SHIFT)
    int32_t p1_x;                   // (I << SHIFT)
    int32_t p1_y;                   // (I << SHIFT)
    int32_t p2_x;                   // (I << SHIFT)
    int32_t p2_y;                   // (I << SHIFT)
    int32_t seg_len;                // (I << SHIFT)
    int32_t seg_spd;                // (IpP << SHIFT)

    // arc
    uint32_t arc_start_date;        // (in microseconds)
    int32_t arc_start_theta;        // (I.rad << SHIFT)
    int32_t c_x;                    // (I << SHIFT)
    int32_t c_y;                    // (I << SHIFT)
    int32_t c_r;                    // (>0 ClockWise | <0 CounterClockWise) (I << SHIFT)
    int32_t arc_len;                // (I << SHIFT)
    int32_t arc_spd;                // (IpP << SHIFT)

    // extra packed data
    uint16_t tid :12;               // original trajectory id
    uint8_t sid :4;                 // original step id
    uint8_t ssid :1;                // first or second element of original message
    int8_t rot1_dir :1;             // sign bit for the rotation 1 (from theta1@p1 to theta2@p2) direction (0: CW | 1: CCW)
    int8_t rot2_dir :1;             // sign bit for the rotation 2 (from theta2@p2 to next theta1@p1) direction (0: CW | 1: CCW)
    enum {
        SLOT_EMPTY,
        SLOT_WAITING_NEXT,
        SLOT_OK
    } state :2;
} sTrajSlot_t;

typedef struct {
    enum {
        S_WAIT, // no action asked (we are stopped)
        S_CHG_TRAJ, // new trajectory to follow
        S_RUN_TRAJ // we are following a trajectory
    } state; // state of the trajectory follow

    // current position
    int x, y; // Robot position on the table (I << SHIFT)
    int theta; // Robot heading on the table (I.rad << SHIFT)
    // current goal
    int gx, gy; // (I << SHIFT)
    int gtheta; // (I.rad << SHIFT)

    sTrajSlot_t traj[TRAJ_MAX_SLOTS]; // circular buffer to store steps of current and next trajectory
} trajectory_manager_t;

void trajmngr_init(trajectory_manager_t* tm);
int trajmngr_new_traj_el(trajectory_manager_t* tm, sTrajOrientElRaw_t *te);
void trajmngr_new_pos(trajectory_manager_t* tm, sPosPayload *pos);

#endif /* TRAJECTORY_MANAGER_H_ */
