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

typedef enum{FRAME_PLAYGROUND, FRAME_PRIMARY, NUM_FRAME}frame_t;

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
    float oz; // angular velocity along z axis of specified reference frame (rad/s)
} s2DSpeed; // 2D linear and angular speeds

typedef struct __attribute__((packed)){
// uncertainty (oriented 2D ellipse)
    float theta_var;  // uncertainty of the 2D orientation (rad²)
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

#endif /* LIB_NETWORK_CONFIG_MESSAGES_POSITION_H_ */
