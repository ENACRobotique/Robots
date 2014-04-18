/*
 * asserv.c
 *
 *  Created on: 26 févr. 2014
 *      Author: ludo6431
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#ifdef ARCH_LPC21XX
#include <lpc214x.h>
#include "gpio.h"
#include "pwm.h"
#include "eint.h"
#include "sys_time.h"
#include "ime.h"
#endif

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../../global_errors.h"
#ifdef ARCH_X86_LINUX
#include "millis.h"
#endif
#include "roles.h"

#include "params.h"
#include "controller.h"
#include "trigo.h"

#include "asserv.h"

#define SQR(v) ((long long)(v)*(v))

#ifdef ARCH_LPC21XX
volatile int _ticks_l=0, _ticks_r=0;

void _isr_left() __attribute__ ((interrupt("IRQ")));
void _isr_left() { // irq17
    if(gpio_read(1, 16))
        _ticks_l--;
    else
        _ticks_l++;

    SCB_EXTINT = BIT(3); // acknowledges interrupt
    VIC_VectAddr = (unsigned)0; // updates priority hardware
}

void _isr_right() __attribute__ ((interrupt("IRQ")));
void _isr_right(){ // irq14
    if(gpio_read(1, 17))
        _ticks_r++;
    else
        _ticks_r--;

    SCB_EXTINT = BIT(0); // acknowledges interrupt
    VIC_VectAddr = (unsigned)0; // updates priority hardware
}
#endif

//#define TIME_STATS

#define TRAJ_MAX_SIZE (16)

typedef struct {
    union {
        struct { // data converted to fixed point (and in increments or in increments per period)
            // segment
            int p1_x;
            int p1_y;
            int p2_x;
            int p2_y;
            int seg_len;  // length of the trajectory on the segment
            // circle
            int c_x;
            int c_y;
            int c_r;
            int arc_len;  // length of the trajectory on the circle
            int arc_sp_max; // max_speed on the circle (radius dependent)
        };
        sTrajElRaw_t raw;
    };
    uint8_t is_ok;
} sTrajEl_t;
sTrajEl_t traj[2][TRAJ_MAX_SIZE];
volatile int curr_traj = 0; // current followed trajectory (0:1)
int curr_traj_step; // current step of the current trajectory (0:curr_traj_insert_sid*2-1)
volatile int curr_traj_insert_sid = 0; // index at which data will be inserted in the current trajectory
volatile int next_traj_insert_sid = 0; // index at which data will be inserted in the next trajectory
volatile uint16_t curr_tid, next_tid; // current and next trajectory identifiers

void traj_conv(sTrajEl_t *t) {
    if(!t->is_ok) {
        t->is_ok = 1;

        t->p1_x = isD2I(t->raw.p1_x);
        t->p1_y = isD2I(t->raw.p1_y);
        t->p2_x = isD2I(t->raw.p2_x);
        t->p2_y = isD2I(t->raw.p2_y);
        t->seg_len = isD2I(t->raw.seg_len);
        t->c_x = isD2I(t->raw.c_x);
        t->c_y = isD2I(t->raw.c_y);
        t->c_r = isD2I(t->raw.c_r);
        t->arc_len = isD2I(t->raw.arc_len);

        t->arc_sp_max = SQRT(1.3*1.3*(double)abs(t->c_r));
    }
}

int x, y, theta; // robot position (I<<SHIFT), robot heading (I.rad<<SHIFT)
int gx, gy; // goal (I<<SHIFT)
int d_consigne = isDpS2IpP(20. /* cm/s */); // desired speed (IpP<<SHIFT)
int _mul_l, _mul_r; // speed multiplier for each wheel (<<SHIFT)
enum {
    S_WAIT, // no action asked (we are stopped)
    S_CHG_TRAJ, // new trajectory to follow
    S_RUN_TRAJ // we are following a trajectory
} state = S_WAIT; // state of the trajectory follow

#ifdef TIME_STATS
// stats
unsigned int m_l, m_c, m_loop;
unsigned int start_us;
#define TIME_STATS_SHIFT (3)
#endif

int ticks_l, ticks_r;
int consigne_l = 0;
int consigne_r = 0;
int ct, st;
int v;
int consigne;
long long tmp, dtime;
int n_x, n_y, i;
int alpha, dist, dist_tmp, dist_lim;

void asserv_init(){
#ifdef ARCH_LPC21XX
    gpio_init_all();  // use fast GPIOs

    pwm_init(0, 1024);  // 29.3kHz update rate => not audible

    // sortie LED
    gpio_output(1, 24);
    gpio_write(1, 24, 0); // green LED on

    gpio_output(0, 31);
    gpio_write(0, 31, 0); // orange LED on

    // entrées quadrature
    gpio_input(1, 16);  // gauche
    gpio_input(1, 17);  // droit

    // interruptions codeurs
    eint_init(_isr_right, _isr_left);
#endif

    motor_controller_init();

#ifdef ARCH_LPC21XX
    // init time management
    sys_time_init();

    global_IRQ_enable();
#endif
}

int new_traj_el(sTrajElRaw_t *te){
    int error = 0;

#ifdef ARCH_X86_LINUX
    printf("got traj step %i|%i\n", te->sid, te->tid);
#endif

    if( curr_traj_insert_sid > 0 && te->tid == curr_tid ) { // we are currently following a trajectory and we got some new steps to add to this trajectory
        if( curr_traj_insert_sid < TRAJ_MAX_SIZE ) {
            if( te->sid == curr_traj_insert_sid ) {
                memcpy(&traj[curr_traj][curr_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
                traj[curr_traj][curr_traj_insert_sid].is_ok = 0;
                curr_traj_insert_sid++;

                state = S_RUN_TRAJ;
            }
            else {
                error = -1; // TODO error: bad step => invalidate all trajectory and ask new one
            }
        }
        else {
            error = -2; // TODO error: too much trajectory steps received
        }
    }
    else if( next_traj_insert_sid > 0 && te->tid == next_tid ) { // we already got some new steps but we still didn't switch to those (next_tid is valid)
        if( next_traj_insert_sid < TRAJ_MAX_SIZE ) {
            if( te->sid == next_traj_insert_sid ) {
                memcpy(&traj[!curr_traj][next_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
                traj[!curr_traj][next_traj_insert_sid].is_ok = 0;
                next_traj_insert_sid++;

                state = S_CHG_TRAJ;
            }
            else {
                error = -3; // TODO error: bad step => invalidate all trajectory and ask new one
            }
        }
        else {
            error = -4; // TODO error: too much trajectory steps received
        }
    }
    else { // we are receiving the first step of a new trajectory
        if( te->sid == 0 ) {
            next_traj_insert_sid = 0;
            next_tid = te->tid;
            memcpy(&traj[!curr_traj][next_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
            traj[!curr_traj][next_traj_insert_sid].is_ok = 0;
            next_traj_insert_sid++;

            state = S_CHG_TRAJ;
        }
        else {
            error = -5; // TODO error: bad step => invalidate all trajectory and ask new one
        }
    }

    //        sb_printDbg(ADDRX_REMOTE_IA, "got traj step, error=", error, 0);

    if( error ) {
        // error...
    }

    return error;
}

int new_pos(sPosPayload *pos){
    if(pos->id == 0) { // prim robot
        x = isD2I(pos->x); // (I<<SHIFT)
        y = isD2I(pos->y); // (I<<SHIFT)
        theta = isROUND( D2I(RDIAM)*pos->theta ); // (rad.I<<SHIFT)
    }

    return 0;
}

int send_pos(){
    sMsg msg;

//    msg.header.destAddr = ADDRD_MONITORING; this is a role_send => the destination address is ignored
    msg.header.type = E_POS;
    msg.header.size = sizeof(msg.payload.pos);
    msg.payload.pos.id = 0; // main robot
    msg.payload.pos.x = I2Ds(x);
    msg.payload.pos.y = I2Ds(y);
    msg.payload.pos.theta = RI2Rs(theta);
    if(state == S_RUN_TRAJ){
        msg.payload.pos.tid = curr_tid;
        msg.payload.pos.sid = curr_traj_step>>1;
        msg.payload.pos.ssid = curr_traj_step&1;
    }
    else{
        msg.payload.pos.tid = msg.payload.pos.sid = -1;
    }

    return role_send(&msg);
}

int new_asserv_step(){
#ifdef TIME_STATS
    start_us = micros();
#endif

    // get current number of ticks per sampling period
#ifdef ARCH_LPC21XX
    global_IRQ_disable();  // prevent _ticks_? to be modified by an interruption caused by an increment
    ticks_l = _ticks_l<<SHIFT; _ticks_l = 0; // (IpP<<SHIFT)
    ticks_r = _ticks_r<<SHIFT; _ticks_r = 0; // (IpP<<SHIFT)
    global_IRQ_enable();
#elif defined(ARCH_X86_LINUX)
    ticks_l = motor_getticks(&motGauche)<<SHIFT; // (IpP<<SHIFT)
    ticks_r = motor_getticks(&motDroit)<<SHIFT; // (IpP<<SHIFT)
#endif

    // update the motor speed
    // TODO: use the shifted data in the PID of the motor, really necessary?
    motor_controller_update(consigne_l>>SHIFT, ticks_l>>SHIFT, consigne_r>>SHIFT, ticks_r>>SHIFT);

    // update speed and course
    v = (ticks_r + ticks_l)>>1; // (IpP<<SHIFT)
    theta += ticks_r - ticks_l; // (rad.I<<SHIFT)
    // get theta's principal angle value
    if(theta > isRPI)
        theta -= (isRPI<<1);
    else if(theta < -isRPI)
        theta += (isRPI<<1);
    ct = COS(theta/iD2I(RDIAM));
    st = SIN(theta/iD2I(RDIAM));

    // update position
    x += ((long long)v * ct)>>SHIFT;
    y += ((long long)v * st)>>SHIFT;

    switch(state) {
    default:
    case S_WAIT:
        consigne_l = 0;
        consigne_r = 0;
        break;

    case S_CHG_TRAJ:
        // current trajectory
        curr_traj = !curr_traj;
        curr_tid = next_tid;
        curr_traj_insert_sid = next_traj_insert_sid;
        next_traj_insert_sid = 0;
        // current state
        state = S_RUN_TRAJ;
        curr_traj_step = 0;

        // speed
        _mul_l = 1<<SHIFT;
        _mul_r = 1<<SHIFT;

        traj_conv(&traj[curr_traj][curr_traj_step>>1]);

        // current goal
        gx = traj[curr_traj][curr_traj_step>>1].p2_x;
        gy = traj[curr_traj][curr_traj_step>>1].p2_y;
        /* no break, fall through to save 20ms */
    case S_RUN_TRAJ:
        // distance to the next goal
        dist = SQRT( (SQR(gy - y) + SQR(gx - x))>>SHIFT );

        // distance needed to stop (knowing the current speed and with a margin of a factor 3!)
        dist_lim = (int)( ((long long)SQR(v)*3/(long long)AMAX)>>SHIFT );

        if(curr_traj_step&1) { // portion de cercle
            consigne = MIN(traj[curr_traj][curr_traj_step>>1].arc_sp_max, d_consigne);
            // TODO (use arc_len - len_travelled)
        }
        else { // portion de segment
            consigne = d_consigne;
            for(
                    i = curr_traj_step>>1, dist_tmp = dist;
                    i < curr_traj_insert_sid && dist_tmp < dist_lim;
                    ++i,
                    traj_conv(&traj[curr_traj][i]),
                    dist_tmp += traj[curr_traj][i].seg_len + traj[curr_traj][i-1].arc_len
            ) {
                dtime = llabs(((long long)v - (long long)traj[curr_traj][i].arc_sp_max)*3/(long long)AMAX);
                if(dtime>0){
                    consigne = MIN(consigne, (int)( ((long long)dist_tmp<<SHIFT)/(long long)dtime ));
                }
            }
        }

        if(dist < isD2I(1)) { // we are near the goal
            if(!(curr_traj_step&1) && abs(traj[curr_traj][curr_traj_step>>1].c_r) < isD2I(1)) { // no next step
                consigne_l = 0;
                consigne_r = 0;
                state = S_WAIT;
                return 0;
            }
            else {
                if( ((curr_traj_step+1)>>1) >= curr_traj_insert_sid ){ // we do not have any steps...
                    consigne_l = 0;
                    consigne_r = 0;
                    return 0;
                }
                curr_traj_step++;
                traj_conv(&traj[curr_traj][(curr_traj_step>>1) + 1]);

                if(curr_traj_step&1) {
                    // circle
                    gx = traj[curr_traj][(curr_traj_step>>1) + 1].p1_x;
                    gy = traj[curr_traj][(curr_traj_step>>1) + 1].p1_y;
                    // set speed a priori
                    _mul_l = ((long long)(traj[curr_traj][curr_traj_step>>1].c_r + isD2I(RDIAM/2.))<<SHIFT)/traj[curr_traj][curr_traj_step>>1].c_r;
                    _mul_r = ((long long)(traj[curr_traj][curr_traj_step>>1].c_r - isD2I(RDIAM/2.))<<SHIFT)/traj[curr_traj][curr_traj_step>>1].c_r;
                }
                else {
                    // straight line
                    gx = traj[curr_traj][curr_traj_step>>1].p2_x;
                    gy = traj[curr_traj][curr_traj_step>>1].p2_y;

                    // set speed a priori
                    _mul_l = 1<<SHIFT;
                    _mul_r = 1<<SHIFT;
                }
            }
        }

        if(curr_traj_step&1) { // circle
            // future error
            n_x = x + ((long long)(v*ct)>>(SHIFT-2)); // position prediction in 4 periods
            n_y = y + ((long long)(v*st)>>(SHIFT-2));
            tmp = -( (SQR(n_x - traj[curr_traj][curr_traj_step>>1].c_x) + SQR(n_y - traj[curr_traj][curr_traj_step>>1].c_y))/traj[curr_traj][curr_traj_step>>1].c_r - traj[curr_traj][curr_traj_step>>1].c_r );

            tmp>>=2;  // gain

#ifdef TIME_STATS
            // time stats... result 160µs! (outdated)
            m_c = m_c - (m_c >> TIME_STATS_SHIFT) + (micros() - start_us);
#endif
        }
        else {  // straight line
            // TODO use same method as in circle follow

            // get principal absolute angle from current position to target
            alpha = iD2I(RDIAM)*ATAN2(gy - y, gx - x);

            // get relative angle to target :  absolute angle to target - current absolute orientation
            tmp =  alpha - theta;  // (rad.I<<SHIFT)
            if(tmp > isRPI) {
                tmp -= (isRPI<<1);
            }
            else if(tmp < -isRPI) {
                tmp += (isRPI<<1);
            }

            tmp>>=4; // gain

#ifdef TIME_STATS
            // time stats... result 257µs! (outdated)
            m_l = m_l - (m_l >> TIME_STATS_SHIFT) + (micros() - start_us);
#endif
        }

        //        if(abs(tmp) > consigne) { // very high compensation term
        //          mybreak_i();
        //        }

        if(abs(tmp) > abs(consigne)>>2){
            consigne = (SQR(consigne)>>2)/abs(tmp);
        }

        // update set point of each motor
        consigne_l = (((long long)_mul_l*(long long)consigne)>>SHIFT) - tmp; // (IpP<<SHIFT)
        consigne_r = (((long long)_mul_r*(long long)consigne)>>SHIFT) + tmp;
        break;
    }

#ifdef TIME_STATS
    // time stats... result 22µs! (in ASSERV_TEST)
    m_loop = m_loop - (m_loop >> TIME_STATS_SHIFT) + (micros() - start_us);
#endif

    return 0;
}

int show_stats(){
#ifdef TIME_STATS
    int ret;
    ret = bn_printfDbg("time stats: circle%uµs, line%uµs, loop%uµs\n", (uint16_t)m_c>>TIME_STATS_SHIFT, (uint16_t)m_l>>TIME_STATS_SHIFT, (uint16_t)m_loop>>TIME_STATS_SHIFT);
    if(ret == -ERR_BN_UNKNOWN_ADDR){
        ret = 0;
    }
    return ret;
#else
    return 0;
#endif
}