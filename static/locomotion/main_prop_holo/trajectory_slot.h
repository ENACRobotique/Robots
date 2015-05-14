/*
 * trajectory_tools.h
 *
 *  Created on: 9 mars 2015
 *      Author: ludo6431
 */

#ifndef TRAJECTORY_SLOT_H_
#define TRAJECTORY_SLOT_H_

#include <stdint.h>
#include "messages-locomotion.h"

// data converted to fixed point (and in increments or in increments per period)
typedef struct {
    // segment
    uint32_t seg_start_date;        // (in microseconds)
    int32_t seg_start_theta;        // (rad << (RAD_SHIFT + SHIFT))
    int32_t p1_x;                   // (I << SHIFT)
    int32_t p1_y;                   // (I << SHIFT)
    int32_t p2_x;                   // (I << SHIFT)
    int32_t p2_y;                   // (I << SHIFT)
    int32_t seg_len;                // (I << SHIFT)
    int32_t seg_spd;                // (IpP << SHIFT)

    // arc
    uint32_t arc_start_date;        // (in microseconds)
    int32_t arc_start_theta;        // (rad << (RAD_SHIFT + SHIFT))
    int32_t c_x;                    // (I << SHIFT)
    int32_t c_y;                    // (I << SHIFT)
    int32_t c_r;                    // (>0 ClockWise | <0 CounterClockWise) (I << SHIFT)
    int32_t arc_len;                // (I << SHIFT)
    int32_t arc_spd;                // >=0 (IpP << SHIFT)

    // extra packed data
    union {
        struct __attribute__((packed)) {
            uint16_t tid :12;               // original trajectory id
            uint8_t sid :5;                 // original step id on 4 MSB  +  lsb: first:0 or second:1 element of original message
        };
        uint32_t id :17;
    };
    int8_t rot1_dir :1;             // sign bit for the rotation 1 (from theta1@p1 to theta2@p2) direction (0: CW | 1: CCW)
    int8_t rot2_dir :1;             // sign bit for the rotation 2 (from theta2@p2 to next theta1@p1) direction (0: CW | 1: CCW)
    int8_t is_last_element :1;      // true if last element of trajectory
    enum {
        SLOT_EMPTY,
        SLOT_WAITING_NEXT,
        SLOT_OK
    } state :2;
} sTrajSlot_t;

void trajslot_update_with_next(sTrajSlot_t* curr, const sTrajSlot_t* next);
int trajslot_create_from_msg(const sTrajOrientElRaw_t* m, sTrajSlot_t* s1, sTrajSlot_t* s2);

#endif /* TRAJECTORY_SLOT_H_ */
