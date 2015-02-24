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

// deprecated, use sTrajOrientElRaw_t
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
} sTrajElRaw_t;

typedef struct {
	// time constraints
	uint32_t t1; // time at elts[0].p1 (synchronized time in us)
	uint16_t t2; // time at elts[0].p2 (offset wrt t1 in ms)
	uint16_t t3; // time at elts[1].p1 (offset wrt t2 in ms)
	uint16_t t4; // time at elts[1].p2 (offset wrt t3 in ms)

	// full trajectory data
    uint16_t tid :12; // trajectory identifier (4096 trajectory id-s)
    uint16_t sid :4; // step identifier (may loop)

	struct __attribute__((packed)) {
	// trajectory
		// segment
		int16_t p1_x; // start point line x coordinate in playground reference frame (cm<<6)
		int16_t p1_y; // start point line y coordinate in playground reference frame (cm<<6)
		int16_t p2_x; // end point line x coordinate in playground reference frame (cm<<6)
		int16_t p2_y; // end point line y coordinate in playground reference frame (cm<<6)

		// circle arc
		int16_t c_x; // circle x coordinate in playground reference frame (cm<<6)
		int16_t c_y; // circle y coordinate in playground reference frame (cm<<6)
		int16_t c_r; // circle radius (cm<<6 ; >0 CW | <0 CCW)

	// orientation
		int16_t theta1; // orientation at p1 (rad<<13)
		int16_t theta2; // orientation at p2 (rad<<13)

	// extra packed data
		int16_t seg_len :15; // segment length (cm<<5)
		int16_t rot1_dir :1; // sign bit for the rotation 1 (from theta1@p1 to theta2@p2) direction (0: CW | 1: CCW)
		int16_t arc_len :15; // arc length (cm<<5)
		int16_t rot2_dir :1; // sign bit for the rotation 2 (from theta2@p2 to next theta1@p1) direction (0: CW | 1: CCW)
	} elts[2]; // 22x2 bytes
} sTrajOrientElRaw_t; // exactly 56 bytes

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
