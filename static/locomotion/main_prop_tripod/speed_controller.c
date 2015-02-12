/*
 * speed_controller.c
 *
 *  Created on: 11 févr. 2015
 *      Author: yoyo
 */
#include "speed_controller.h"
#include "encoder.h"
#include "param.h"
#include "ime.h"


void get_enc_pv(speed_controller_t* spd_ctl, encoder_t** tab_enc){
    int i;
    global_IRQ_disable(); // Prevent nbticks to be modified by an interruption caused by an increment
    for(i=0; i<3; i++){
        spd_ctl->speeds_pv[i] = get_encoder(&tab_enc[i])/PER_ASSER;
        *tab_enc[i]->p_nbticks = *tab_enc[i]->nbticks;
    }
    global_IRQ_enable();
}
