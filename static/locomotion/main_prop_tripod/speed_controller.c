/*
 * speed_controller.c
 *
 *  Created on: 11 févr. 2015
 *      Author: yoyo
 */
#include "speed_controller.h"
#include "encoder.h"
#include "param.h"


void get_speeds_pv(speed_controller_t* spd_ctl, encoder_t** tab_enc){
    int i;
    for(i=0; i<3; i++){
        spd_ctl->speeds_pv[i] = get_encoder(&tab_enc[i])/PER_ASSER;
    }
}
