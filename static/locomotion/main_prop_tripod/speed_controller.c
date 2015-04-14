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

    pid_init(&sc->pid, 1 << SHIFT_PID_SPD, 1 << (SHIFT_PID_SPD - 2), 0, 900 << SHIFT_PID_SPD, SHIFT_PID_SPD);
}

void spdctlr_update(speed_controller_t* sc, int setpoint) {
    int processValue = encoder_get(sc->enc);

    sc->cmd_cache = pid_update(&sc->pid, setpoint, processValue);
}

int spdctlr_get(speed_controller_t* sc) {
    return sc->cmd_cache;
}
