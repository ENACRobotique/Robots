/*
 * speed_controller.h
 *
 *  Created on: 11 févr. 2015
 */

#ifndef SPEED_CONTROLLER_H_
#define SPEED_CONTROLLER_H_

#include <encoder.h>
#include <motor.h>
#include <pid.h>
#include "lowpass_filter.h"

typedef struct {
    encoder_t* enc;

    PID_t pid;
    lowpass_t lp;
    int cmd_cache;

    // XXX
    int lastSP;
} speed_controller_t;

void spdctlr_init(speed_controller_t* sc, encoder_t* enc);
void spdctlr_update(speed_controller_t* sc, int setpoint);
int spdctlr_get(speed_controller_t* sc);

#endif /* SPEED_CONTROLLER_H_ */
