/*
 * trajectory_controller.h
 *
 *  Created on: 11 févr. 2015
 *      Author: yoyo
 */

#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include <matrix.h>

typedef struct{
    MAT *geometry;

    int x, y;

    // data for trajectory control

    // data for orientation control

    encoder_t enc[3];
    motor_t m[3];
    speed_controller_t spd_ctls[3];
} trajectory_controller_t;

#endif /* TRAJECTORY_CONTROLLER_H_ */
