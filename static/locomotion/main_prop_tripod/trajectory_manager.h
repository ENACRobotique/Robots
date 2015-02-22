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

#define TRAJ_MAX_SIZE (16)

void trajmngr_new_speed_sp(float speed);
int trajmngr_new_traj_el(sTrajElRaw_t *te);
void trajmngr_new_pos(sPosPayload *pos);

#endif /* TRAJECTORY_MANAGER_H_ */
