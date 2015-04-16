/*
 * speed_controller.c
 *
 *  Created on: 11 févr. 2015
 */

#include <stdint.h>

#include "params.h"

#include <speed_controller.h>

#ifndef ABS
#   define ABS(v) ((v)>0?(v):-(v))
#endif
#ifndef SIGN0
// sign of value being 0 when value is zero
#   define SIGN0(v) (((v)>0) - ((v)<0))
#endif
#define SHIFT_PID_EXT (8)
#define SHIFT_LP (3)

void spdctlr_init(speed_controller_t* sc, encoder_t* enc) {
    sc->enc = enc;
    sc->cmd_cache = 0;

#define dPSHIFT ((double)(1 << SHIFT_PID_SPD))

    pid_init(&sc->pid, iROUND(0.45 * dPSHIFT), iROUND(0.54 * dPSHIFT), iROUND(0 * dPSHIFT), 500 << (SHIFT_PID_SPD + SHIFT_PID_EXT), SHIFT_PID_SPD);

    // for a value of 2, it takes  8 samples to get from 10% to 90% (rise-time)
    // for a value of 3, it takes 16 samples to get from 10% to 90% (rise-time)
    // for a value of 4, it takes 34 samples to get from 10% to 90% (rise-time)
    lp_init(&sc->lp, SHIFT_LP);
}

// computes an estimation of the motor command (with static friction in the middle):
//    (3/4*abs(setpoint) + 26) * sign(setpoint)
int sp2cmd(int setpoint) {
    int64_t absSP = ABS(setpoint);

    int64_t cmd = (3ll * absSP) >> 2; // 0.75*setpoint
    return ((int32_t)cmd + 26) * SIGN0(setpoint);
}

// computes a weighting factor for the pid part of the resulting cmd
int64_t sp2weight(speed_controller_t* sc, int setpoint) {
    lp_update(&sc->lp, ABS(setpoint));
    int64_t lpsp = lp_getFullRes(&sc->lp);

#define MIN_FACTOR (3 << (SHIFT_PID_EXT - 2)) // 75%
#define MAX_FACTOR (1 << SHIFT_PID_EXT) // 100%
#define MIN_SP (10 << SHIFT_LP)
#define MAX_SP (25 << SHIFT_LP)

    int64_t factor;

    if(lpsp <= MIN_SP) {
        factor = MIN_FACTOR;
    }
    else if(lpsp >= MAX_SP) {
        factor = MAX_FACTOR;
    }
    else { // linear from min to max
        factor = MIN_FACTOR + (MAX_FACTOR - MIN_FACTOR)*(lpsp - MIN_SP)/(MAX_SP - MIN_SP);
    }

#undef MIN_FACTOR
#undef MAX_FACTOR
#undef MIN_SP
#undef MAX_SP

    return factor;
}

void spdctlr_update(speed_controller_t* sc, int setpoint) {
    int processValue = encoder_get(sc->enc);

    int64_t pidRes = pid_update(&sc->pid, setpoint << SHIFT_PID_EXT, processValue << SHIFT_PID_EXT); // (in <<SHIFT_PID_EXT)

    int64_t weight = sp2weight(sc, setpoint); // (in <<SHIFT_PID_EXT)

    sc->lastSP = setpoint;

    sc->cmd_cache = sp2cmd(setpoint) + ((weight * pidRes) >> (SHIFT_PID_EXT*2));
}

int spdctlr_get(speed_controller_t* sc) {
    return sc->cmd_cache;
}
