#include <stdlib.h> // abs()

#include "pid.h"
#include "params.h"

#include "controller.h"

PID_t vitMotGauche, vitMotDroit;
motor_t motGauche, motDroit;

//#define ASSERV_STATS

#ifdef ASSERV_STATS
#include <string.h>
#include "shared/botNet_core.h"
#include "shared/bn_debug.h"
#include "millis.h"
#include "messages.h"

sMsg outMsg;
#define AS (&outMsg.payload.asservStats)
int i_as = 0;
unsigned int prev_time = 0;
unsigned int nb_seq = 0;
#endif

void motor_controller_init() {
    // asserv
    pid_init(&vitMotGauche, 2<<SHIFT_PID, (2*2/1)<<(SHIFT_PID-3), /*(2*2/1)<<(SHIFT_PID-3)*/0, 900<<SHIFT_PID, SHIFT_PID);
    pid_init(&vitMotDroit, 2<<SHIFT_PID, (2*2/1)<<(SHIFT_PID-3), /*(2*2/1)<<(SHIFT_PID-3)*/0, 900<<SHIFT_PID, SHIFT_PID);

    // motors
    motor_init(&motGauche, 4 /* P0.8 */, 1, 22);
    motor_init(&motDroit,  6 /* P0.9 */, 1, 23);

#ifdef ASSERV_STATS
    prev_time = micros();
#endif
}

void motor_controller_update(int sPL, int pVL, int sPR, int pVR) {
    int valL, valR;
    int errL, errR, newerr;

#define SIGN(v) ((v)>=0?1:-1)

#ifdef ASSERV_STATS
    {
        unsigned int t = micros();
        AS->steps[i_as].delta_t = t - prev_time;
        prev_time = t;
    }
#endif

    // limit acceleration (keeping ratio of errors constant)
    errL = sPL - pVL;
    errR = sPR - pVR;
    if(abs(errL) > abs(errR) && abs(errL) > AMAX){
        newerr = SIGN(errL)*AMAX;
        sPL = pVL + newerr;
        sPR = pVR + newerr*errR/errL;
    }
    else if(abs(errR) > abs(errL) && abs(errR) > AMAX){
        newerr = SIGN(errR)*AMAX;
        sPR = pVR + newerr;
        sPL = pVL + newerr*errL/errR;
    }

    // update PIDs
    valL = pid_update(&vitMotGauche, sPL, pVL);
    valR = pid_update(&vitMotDroit, sPR, pVR);

    // send command to motors
    motor_update(&motGauche, valL);
    motor_update(&motDroit, valR);

#ifdef ASSERV_STATS
    AS->steps[i_as].ticks_l = pVL;
    AS->steps[i_as].consigne_l = sPL;
    AS->steps[i_as].out_l = valL;

    AS->steps[i_as].ticks_r = pVR;
    AS->steps[i_as].consigne_r = sPR;
    AS->steps[i_as].out_r = valR;

    if(i_as >= NB_ASSERV_STEPS_PER_MSG-1){
        i_as = 0;
        AS->nb_seq = nb_seq++;

        outMsg.header.destAddr = role_get_addr(ROLE_DEBUG);
        if(outMsg.header.destAddr){
            outMsg.header.type = E_ASSERV_STATS;
            outMsg.header.size = sizeof(outMsg.payload.asservStats);

            bn_send(&outMsg);
        }
    }
    else{
        i_as++;
    }
#endif
}
