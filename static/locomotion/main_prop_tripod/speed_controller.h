/*
 * speed_controller.h
 *
 *  Created on: 11 févr. 2015
 *      Author: yoyo
 */

#ifndef SPEED_CONTROLLER_H_
#define SPEED_CONTROLLER_H_

#include "encoder.h"

typedef struct{
    int speeds_pv[3];
    int speeds_cmd[3];

} speed_controller_t;

void get_speeds_pv(speed_controller_t* spd_ctl, encoder_t** tab_enc);

#endif /* SPEED_CONTROLLER_H_ */
