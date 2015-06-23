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
#include "trajectory_slot.h"
#include "position_controller.h"

#define TRAJ_MAX_SLOTS (32)

typedef struct {
    position_controller_t ctlr;

    enum {
        TM_STATE_IDLE, // no control loop on position
        TM_STATE_WAIT_TRAJ, // no action asked (we are stopped)
        TM_STATE_WAIT_START, // new trajectory received, waiting for the right time to start
        TM_STATE_FOLLOWING // we are following a trajectory
    } state; // state of the trajectory follow

    uint16_t curr_tid :12;
    uint8_t next_sid :4;
    uint16_t curr_element; // slot index + sub step id
    uint8_t curr_rid; // current recalibration id (initialized to 0)

    uint16_t slots_insert_idx;
    uint16_t slots_used_number;
    sTrajSlot_t slots[TRAJ_MAX_SLOTS]; // circular buffer to store steps of current and next trajectory

    struct {
        uint32_t id :17; // see id of a sTrajSlot_t
        enum {
            TM_CACHE_STATE_EMPTY,
            TM_CACHE_STATE_LINE,
            TM_CACHE_STATE_ARC
        } state;

        union {
            struct {
                int spd_x; // (IpP << SHIFT)
                int spd_y; // (IpP << SHIFT)

                int dur; // (periods)
            } line;
            struct {
                int omega_z; // (<0 ClockWise | >0 CounterClockWise) (in radpP << (RAD_SHIFT+SHIFT))

                int dur; // (periods)
            } arc;
        };
    } cache;

    // current goal
    int gx, gy; // (in I << SHIFT)
    int gtheta; // (in R << (RAD_SHIFT + SHIFT))
} trajectory_manager_t;

void trajmngr_init(trajectory_manager_t* tm, const int32_t mat_rob2pods[NB_PODS][NB_SPDS]);
void trajmngr_reset(trajectory_manager_t* tm);
int trajmngr_new_traj_el(trajectory_manager_t* tm, const sTrajOrientElRaw_t *te);
void trajmngr_set_pos(trajectory_manager_t* tm, const sGenericPosStatus *pos);
void trajmngr_mix_pos(trajectory_manager_t* tm, const sGenericPosStatus *pos);
void trajmngr_get_pos_status(trajectory_manager_t* tm, sGenericPosStatus *ps);
int trajmngr_update(trajectory_manager_t* tm);
void trajmngr_stop(trajectory_manager_t* tm);

#endif /* TRAJECTORY_MANAGER_H_ */
