/*
 * speed_controller.c
 *
 *  Created on: 11 févr. 2015
 */

#include <speed_controller.h>

#define SHIFT_PID (8)

void spdctl_init(speed_controller_t* sc, encoder_t* enc) {
    sc->enc = enc;

    pid_init(&sc->pid, 2<<SHIFT_PID, (2*2/1)<<(SHIFT_PID-3), /*(2*2/1)<<(SHIFT_PID-3)*/0, 900<<SHIFT_PID, SHIFT_PID);
}

int spdctl_update(speed_controller_t* sc, int setpoint) {
    int processValue = encoder_get(sc->enc);

    return pid_update(&sc->pid, setpoint, processValue);
}
