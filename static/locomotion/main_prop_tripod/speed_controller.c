/*
 * speed_controller.c
 *
 *  Created on: 11 févr. 2015
 */

#include "params.h"

#include <speed_controller.h>

void spdctlr_init(speed_controller_t* sc, encoder_t* enc) {
    sc->enc = enc;
    sc->cmd_cache = 0;

//    pid_init(&sc->pid, 1 << (SHIFT_PID_SPD - 1), 1 << (SHIFT_PID_SPD - 1), 0, 900 << SHIFT_PID_SPD, SHIFT_PID_SPD);

#define dPSHIFT ((double)(1 << SHIFT_PID_SPD))

//    pid_init(&sc->pid, iROUND(0.6666 * dPSHIFT), iROUND(0.83333 * dPSHIFT), iROUND(0.075 * dPSHIFT), 500 << SHIFT_PID_SPD, SHIFT_PID_SPD);
    pid_init(&sc->pid, iROUND(0.8*0.6 * dPSHIFT), iROUND(0.8*0.73333 * dPSHIFT), iROUND(0.8*0.06 * dPSHIFT), 500 << SHIFT_PID_SPD, SHIFT_PID_SPD);
}

void spdctlr_update(speed_controller_t* sc, int setpoint) {
    int processValue = encoder_get(sc->enc);

    sc->cmd_cache = pid_update(&sc->pid, setpoint, processValue);
}

int spdctlr_get(speed_controller_t* sc) {
    return sc->cmd_cache;
}
