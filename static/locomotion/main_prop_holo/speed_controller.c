/*
 * speed_controller.c
 *
 *  Created on: 11 févr. 2015
 */

#include <stdint.h>

#include "params.h"

#include <speed_controller.h>

void spdctlr_init(speed_controller_t* sc, encoder_t* enc) {
    sc->enc = enc;
    sc->cmd_cache = 0;

//    pid_init(&sc->pid, 1 << (SHIFT_PID_SPD - 1), 1 << (SHIFT_PID_SPD - 1), 0, 900 << SHIFT_PID_SPD, SHIFT_PID_SPD);

#define dPSHIFT ((double)(1 << SHIFT_PID_SPD))

    pid_init(&sc->pid, iROUND(0.45 * dPSHIFT), iROUND(0.54 * dPSHIFT), iROUND(0 * dPSHIFT), 500 << SHIFT_PID_SPD, SHIFT_PID_SPD);
}

#ifndef ABS
#   define ABS(v) ((v)>0?(v):-(v))
#endif
#ifndef SIGN0
// sign of value being 0 when value is zero
#   define SIGN0(v) (((v)>0) - ((v)<0))
#endif

// computes an estimation of the motor command (with static friction in the middle):
//    (3/4*abs(setpoint) + 26) * sign(setpoint)
int sp2cmd(int setpoint) {
    int64_t absSP = ABS(setpoint);

    int64_t cmd = (3ll * absSP) >> 2; // 0.75*setpoint
    return ((int32_t)cmd + 26) * SIGN0(setpoint);
}

void spdctlr_update(speed_controller_t* sc, int setpoint) {
    int processValue = encoder_get(sc->enc);

    sc->cmd_cache = sp2cmd(setpoint) + pid_update(&sc->pid, setpoint, processValue);
}

int spdctlr_get(speed_controller_t* sc) {
    return sc->cmd_cache;
}
