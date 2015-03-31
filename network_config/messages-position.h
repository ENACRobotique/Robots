/*
 * messages-position.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_POSITION_H_
#define LIB_NETWORK_CONFIG_MESSAGES_POSITION_H_

#include <messages-elements.h>
#include <stdint.h>

typedef enum{FRAME_PLAYGROUND, FRAME_PRIMARY}frame_t;

typedef struct __attribute__((packed)){
// position in reference frame (specified in field "frame")
    float x;            // (cm)
    float y;            // (cm)
    float theta;        // (rad)
    frame_t frame:8;
} s2DPosAtt; // 2D position & attitude

typedef struct __attribute__((packed)){
// speed in reference frame
    float vx; // linear velocity along x axis of specified reference frame (cm/s)
    float vy; // linear velocity along y axis of specified reference frame (cm/s)
    float oz; // angular velocity along z axis of specified reference frame (cm/s)
} s2DSpeed; // 2D linear and angular speeds

typedef struct __attribute__((packed)){
// uncertainty (oriented 2D ellipse)
    float theta;      // uncertainty of the 2D orientation (rad)
    float a_angle;    // orientation of the uncertainty ellipse along "a" axis (rad)
    float a_var;      // variance along "a" axis (cm²)
    float b_var;      // variance along "b" axis (cm²)
} s2DPAUncert;

typedef struct{
    uint32_t date; // synchronized date (µs)
    eElement id :8;
} sPosQuery;

#define NB_POS_STEPS_PER_MSG (7)
typedef struct __attribute__((packed)){
    uint16_t nb_seq;
    struct __attribute__((packed)){ // 7bytes
        unsigned short delta_t; // (µs)
        uint16_t x :14; // 0-16383 (quarter of mm)
        uint16_t y :14; // 0-16383 (quarter of mm)
        uint16_t theta :12; // 0-4095 (tenth of °)
    }steps[NB_POS_STEPS_PER_MSG];
} sPosStats;

typedef struct {
// position in game area frame
    float x; // (cm)
    float y; // (cm)
    float theta; // (rad)
// uncertainty (oriented rectangle)
    float u_a_theta; // (rad)
    float u_a; // (cm)
    float u_b; // (cm)
// trajectory steps
    int tid; // trajectory identifier               // FIXME int -> intxx_t
    int sid; // step identifier                     // FIXME int -> intxx_t
    uint8_t ssid; // sub-step identifier (0:line, 1:circle)
// identifier of the robot/element
    uint8_t id; // 0:prim, 1:sec, 2:adv_prim, 3:adv_sec
} sPosPayload; // XXX DEPRECATED use sGenericStatus instead

#endif /* LIB_NETWORK_CONFIG_MESSAGES_POSITION_H_ */
