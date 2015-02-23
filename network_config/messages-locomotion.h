/*
 * messages-position.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_LOCOMOTION_H_
#define LIB_NETWORK_CONFIG_MESSAGES_LOCOMOTION_H_

#include <stdint.h>

typedef struct{
    float speed;
} sSpeedSetPoint;

typedef struct {
// segment (centimeters)
    float p1_x;
    float p1_y;
    float p2_x;
    float p2_y;
    float seg_len;
// circle
    float c_x;
    float c_y;
    float c_r; // >0 CW | <0 CCW
    float arc_len;
// trajectory data
    uint16_t tid; // trajectory identifier
    uint16_t sid; // step identifier
    float cumul_cvdist; // cumulative curvilinear distance before this step (centimeters)
} sTrajElRaw_t;

typedef struct {
// start point
    float p1_s; // traveled distance (aka curvilinear abscissa)
    float p1_a; // orientation (rad)
    float seg_cvlen; // curvilinear length of this orientation step
// orientation data
    uint16_t tid; // orientation identifier
    uint16_t sid; // step identifier
    float cumul_cvdist; // cumulative curvilinear distance before this step (centimeters)
} sOrientElRaw_t;

#define NB_ASSERV_STEPS_PER_MSG (4)
typedef struct __attribute__((packed)){
    uint16_t nb_seq;
    struct __attribute__((packed)){ // 13bytes*4
        unsigned short delta_t;
        short ticks_l;
        short ticks_r;
        short consigne_l;
        short consigne_r;
        short out_l :12;
        short out_r :12;
    } steps[NB_ASSERV_STEPS_PER_MSG];
} sAsservStats;

#endif /* LIB_NETWORK_CONFIG_MESSAGES_LOCOMOTION_H_ */
