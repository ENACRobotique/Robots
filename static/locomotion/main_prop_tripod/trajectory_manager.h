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
#include "trajectory_controller.h"

#define TRAJ_MAX_SLOTS (32)

typedef struct {
    trajectory_controller_t ctlr;

    enum {
        TM_STATE_WAIT, // no action asked (we are stopped)
        TM_STATE_FOLLOWING // we are following a trajectory
    } state; // state of the trajectory follow

    uint16_t curr_tid :12;
    uint8_t next_sid :4;
    uint16_t curr_slot_idx; // slot index + sub sub step id

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
                int omega_z; // (in radpP << (RAD_SHIFT + SHIFT))

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
void trajmngr_set_pos(trajectory_manager_t* tm, const sPosPayload *pos);
int trajmngr_update(trajectory_manager_t* tm);

#endif /* TRAJECTORY_MANAGER_H_ */
