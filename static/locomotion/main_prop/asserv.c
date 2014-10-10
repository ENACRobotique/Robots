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
#include "backlash.h"
#endif

#include "../botNet/shared/botNet_core.h"
#include "../network_tools/bn_debug.h"
#include "../../../global_errors.h"
#ifdef ARCH_X86_LINUX
#include "millis.h"
#endif
#include "roles.h"

#include "params.h"
#include "controller.h"
#include "trigo.h"
#include "pid.h"

#include "asserv.h"

#define SQR(v) ((long long)(v)*(v))
#define SIGN(v) (((v)>0) - ((v)<0))

#ifdef ARCH_LPC21XX
volatile int _ticks_l=0, _ticks_r=0;

void _isr_left() __attribute__ ((interrupt("IRQ")));
void _isr_left() { // irq17
    if(gpio_read(0, 22))
        _ticks_l--;
    else
        _ticks_l++;

    SCB_EXTINT = BIT(3); // acknowledges interrupt
    VIC_VectAddr = (unsigned)0; // updates priority hardware
}

void _isr_right() __attribute__ ((interrupt("IRQ")));
void _isr_right(){ // irq14
    if(gpio_read(0, 18))
        _ticks_r++;
    else
        _ticks_r--;

    SCB_EXTINT = BIT(0); // acknowledges interrupt
    VIC_VectAddr = (unsigned)0; // updates priority hardware
}

sBackLash blLeft, blRight;
#endif

//#define TIME_STATS
//#define ASSERV_LOGS
//#define POS_STATS
//#define POS_USE_PID

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
#ifdef POS_USE_PID
PID_t posLeft, posRight;
#endif

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

int x = 0, y = 0, theta = 0; // robot position (I<<SHIFT), robot heading (I.rad<<SHIFT)
unsigned int t_pos = 0; // time corresponding to the position
int gx = 0, gy = 0, gtheta = 0; // goal (I<<SHIFT)
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

#ifdef POS_STATS
sMsg ps_outMsg;
#define PS (&ps_outMsg.payload.posStats)
int ps_i = 0;
unsigned int ps_prevTime = 0;
unsigned int ps_nbSeq = 0;
#endif

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
    gpio_input(0, 22);  // gauche
    gpio_input(0, 18);  // droit

    // interruptions codeurs
    eint_disable(EINT0);
    eint_assign(EINT0_P0_16);
    eint_mode(EINT0, EINT_RISING_EDGE);
    eint_register(EINT0, _isr_right, 2);
    eint_enable(EINT0);

    eint_disable(EINT3);
    eint_assign(EINT3_P0_20);
    eint_mode(EINT3, EINT_RISING_EDGE);
    eint_register(EINT3, _isr_left, 3);
    eint_enable(EINT3);

    // backlash compensation
    backlash_init(&blLeft, iR2I(7.295*PI/180.), 0);
    backlash_init(&blRight, iR2I(1.574*PI/180.), 0);
#endif

    motor_controller_init();

#ifdef POS_USE_PID
    pid_init(&posLeft, 1<<(SHIFT_PID-8), /*(2*2/1)<<(SHIFT_PID-3)*/0, /*(2*2/1)<<(SHIFT_PID-3)*/0, 900<<SHIFT_PID, SHIFT_PID);
    pid_init(&posRight, 1<<(SHIFT_PID-8), /*(2*2/1)<<(SHIFT_PID-3)*/0, /*(2*2/1)<<(SHIFT_PID-3)*/0, 900<<SHIFT_PID, SHIFT_PID);
#endif

#ifdef ARCH_LPC21XX
    // init time management
    sys_time_init();

    global_IRQ_enable();
#endif

#ifdef POS_STATS
    ps_prevTime = micros();
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

int new_speed_setpoint(float speed){
    d_consigne = isDpS2IpP(speed);
    return 0;
}

int new_pos(sPosPayload *pos){
    if(pos->id == 0) { // prim robot
        x = isD2I(pos->x); // (I<<SHIFT)
        y = isD2I(pos->y); // (I<<SHIFT)
        theta = isROUND( D2I(RDIAM)*pos->theta ); // (rad.I<<SHIFT)

        if(state == S_WAIT){
            gx = x;
            gy = y;
            gtheta = theta;
        }
    }

    return 0;
}

int send_pos(){
    sMsg msg = {{0}};

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

void get_pos(s2DPosAtt *p, s2DPAUncert *p_u, unsigned int *p_t){
    if(p){
        p->frame = FRAME_PLAYGROUND;
        p->x = I2Ds(x);
        p->y = I2Ds(y);
        p->theta = RI2Rs(theta);
    }

    if(p_u){
        // TODO
        memset(p_u, 0, sizeof(*p_u));
    }

    if(p_t){
        *p_t = t_pos;
    }
}

#if defined(ARCH_X86_LINUX) && defined(ASSERV_LOGS)
FILE *fout = NULL;
#endif

int new_asserv_step(){
#ifdef TIME_STATS
    start_us = micros();
#endif

#ifdef POS_STATS
    {
        unsigned int t = micros();
        PS->steps[ps_i].delta_t = t - ps_prevTime;
        ps_prevTime = t;
    }
#endif

    // get current number of ticks per sampling period
#ifdef ARCH_LPC21XX
    global_IRQ_disable();  // prevent _ticks_? to be modified by an interruption caused by an increment
    ticks_l = _ticks_l<<SHIFT; _ticks_l = 0; // (IpP<<SHIFT)
    ticks_r = _ticks_r<<SHIFT; _ticks_r = 0; // (IpP<<SHIFT)
    global_IRQ_enable();

//    ticks_l = backlash_update(&blLeft, ticks_l>>SHIFT)<<SHIFT;
//    ticks_r = backlash_update(&blRight, ticks_r>>SHIFT)<<SHIFT;
#elif defined(ARCH_X86_LINUX)
    ticks_l = motor_getticks(&motGauche)<<SHIFT; // (IpP<<SHIFT)
    ticks_r = motor_getticks(&motDroit)<<SHIFT; // (IpP<<SHIFT)
#endif

    t_pos = micros();

    // update the motor speed
    // TODO: use the shifted data in the PID of the motor, really necessary?
#if defined(ARCH_X86_LINUX)
//    printf("c_l=%i, c_r=%i, t_l=%i, t_r=%i\n\n", consigne_l, consigne_r, ticks_l, ticks_r);
#endif
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

#ifdef POS_STATS
    {
        int it;

        PS->steps[ps_i].x = iROUND(40.*I2Ds((float)x));
        PS->steps[ps_i].y = iROUND(40.*I2Ds((float)y));

        it = theta < 0 ? theta + (isRPI<<1) : theta;
        PS->steps[ps_i].theta = iROUND(10.*RI2Rs((float)it)*180./PI);

        if(ps_i >= NB_POS_STEPS_PER_MSG-1){
            ps_i = 0;
            PS->nb_seq = ps_nbSeq++;

            ps_outMsg.header.destAddr = role_get_addr(ROLE_DEBUG);
            if(ps_outMsg.header.destAddr){
                ps_outMsg.header.type = E_POS_STATS;
                ps_outMsg.header.size = sizeof(ps_outMsg.payload.posStats);

                bn_send(&ps_outMsg);
            }
        }
        else{
            ps_i++;
        }
    }
#endif

#ifdef ARCH_X86_LINUX
//    printf("%i,%i,%i,%i,%i,%i,%i\x1b[K\n", ticks_l, ticks_r, v, theta, isRPI, ct, st);
#endif

    switch(state) {
    default:
    case S_WAIT:
        { // position control loop
            // x (I<<SHIFT), y (I<<SHIFT), theta (rad.I<<SHIFT), ct (1<<SHIFT), st (1<<SHIFT), gx (I<<SHIFT), gy (I<<SHIFT), gtheta (rad.I<<SHIFT)

            int d = ((long)(gx - x)*(long)ct + (long)(gy - y)*(long)st)>>SHIFT;
            int diff_theta = theta - gtheta;

            if(diff_theta > isRPI)
                diff_theta -= (isRPI<<1);
            else if(diff_theta < -isRPI)
                diff_theta += (isRPI<<1);

            int distL = (diff_theta>>1) + d;
            int distR = -(diff_theta>>1) + d;

#ifdef POS_USE_PID
            // update PIDs
            consigne_l = pid_update(&posLeft, 0, distL>>SHIFT);
            consigne_r = pid_update(&posRight, 0, distR>>SHIFT);
#else
            consigne_l = distL>>2;
            consigne_r = distR>>2;
#endif

#if 0 && defined(ARCH_X86_LINUX)
            printf("\n\nx=%.2f, y=%.2f, theta=%.2f°, gx=%.2f, gy=%.2f, gtheta=%.2f°, d=%i, dL=%i, dR=%i, c_l=%i, c_r=%i\n\n", I2Ds(x), I2Ds(y), RI2Rs(theta)*180./PI, I2Ds(gx), I2Ds(gy), RI2Rs(gtheta)*180./PI, d>>SHIFT, distL>>SHIFT, distR>>SHIFT, consigne_l, consigne_r);
            {
                static FILE *flog = NULL;
                if(!flog){
                    flog = fopen("flog.csv", "wb+");

                    if(flog){
                        fprintf(flog, "x,y,theta,ct,st,gx,gy,gtheta,d,dL,dR\n");
                    }
                }

                if(flog){
                    fprintf(flog, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", I2Ds(x), I2Ds(y), RI2Rs(theta)*180./PI, ((float)ct)/dSHIFT, ((float)st)/dSHIFT, I2Ds(gx), I2Ds(gy), RI2Rs(gtheta)*180./PI, I2Ds(d), I2Ds(distL), I2Ds(distR));
                    fflush(flog);
                }
            }
#endif
        }
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
#if defined(ARCH_X86_LINUX) && defined(ASSERV_LOGS)
        if(!fout){
            fout = fopen("out.csv", "w+");
        }
#endif

        // distance needed to stop (knowing the current speed and with a margin of a factor 3!)
        dist_lim = (int)( ((long long)SQR(v)*3/(long long)AMAX)>>SHIFT );

        if(curr_traj_step&1) { // portion de cercle
            int tp, tg;
            long long prod;
            tp = ATAN2(y - traj[curr_traj][curr_traj_step>>1].c_y, x - traj[curr_traj][curr_traj_step>>1].c_x); // (rad<<SHIFT)
            tg = ATAN2(gy - traj[curr_traj][curr_traj_step>>1].c_y, gx - traj[curr_traj][curr_traj_step>>1].c_x); // (rad<<SHIFT)
            prod = ((long long)(tp - tg))*((long long)traj[curr_traj][curr_traj_step>>1].c_r)>>SHIFT;
            dist = abs(prod);

            consigne = SIGN(d_consigne)*MIN(traj[curr_traj][curr_traj_step>>1].arc_sp_max, abs(d_consigne));
            // TODO (use arc_len - len_travelled)
        }
        else { // portion de segment
            // distance to the next goal
            dist = SQRT( (SQR(gy - y) + SQR(gx - x))>>SHIFT );

            consigne = abs(d_consigne);
            for(
                    i = curr_traj_step>>1, dist_tmp = dist;
                    i < curr_traj_insert_sid && dist_tmp < dist_lim;
                    ++i,
                    traj_conv(&traj[curr_traj][i]),
                    dist_tmp += traj[curr_traj][i].seg_len + traj[curr_traj][i-1].arc_len
            ) {
                dtime = llabs(((long long)abs(v) - (long long)traj[curr_traj][i].arc_sp_max)*3/(long long)AMAX);
                if(dtime>0){
                    consigne = MIN(consigne, (int)( ((long long)dist_tmp<<SHIFT)/(long long)dtime ));
                }
            }
            if(d_consigne < 0){
                consigne = -consigne;
            }
        }

        if(dist < isD2I(1)) { // we are near the goal
            if(!(curr_traj_step&1) && abs(traj[curr_traj][curr_traj_step>>1].c_r) < isD2I(1)) { // no next step
                state = S_WAIT;

                gx = traj[curr_traj][curr_traj_step>>1].p2_x;
                gy = traj[curr_traj][curr_traj_step>>1].p2_y;
                if(abs(traj[curr_traj][curr_traj_step>>1].seg_len) < isD2I(1)){ // no segment in current traj step
                    if((curr_traj_step>>1) == 0){ // only one step
                        gtheta = theta;
                    }
                    else{ // get angle with previous traj step
                        gtheta = iD2I(RDIAM)*ATAN2(traj[curr_traj][curr_traj_step>>1].p1_y - traj[curr_traj][(curr_traj_step>>1)-1].c_y, traj[curr_traj][curr_traj_step>>1].p1_x - traj[curr_traj][(curr_traj_step>>1)-1].c_x) - (isRPI>>1)*SIGN(traj[curr_traj][(curr_traj_step>>1)-1].c_r);
                    }
                }
                else{ // get angle from segment
                    gtheta = iD2I(RDIAM)*ATAN2(traj[curr_traj][curr_traj_step>>1].p2_y - traj[curr_traj][curr_traj_step>>1].p1_y, traj[curr_traj][curr_traj_step>>1].p2_x - traj[curr_traj][curr_traj_step>>1].p1_x);
                }
                if(d_consigne < 0){ // in case of backward motion
                    gtheta = gtheta + isRPI;
                }

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
                    if(consigne < 0){
                        int temp = _mul_l;
                        _mul_l = _mul_r;
                        _mul_r = temp;
                    }
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

            if(consigne < 0){
                alpha += isRPI;
            }

            // get relative angle to target :  absolute angle to target - current absolute orientation
            tmp =  alpha - theta;  // (rad.I<<SHIFT)
            if(tmp > isRPI) {
                tmp -= (isRPI<<1);
            }
            else if(tmp < -isRPI) {
                tmp += (isRPI<<1);
            }

#if defined(ARCH_X86_LINUX) && defined(ASSERV_LOGS)
            if(fout){
                fprintf(fout, "%u,0,%i,%i,%lli,%f,%f,%f\n", millis(), alpha, theta, tmp, RI2Rs(alpha), RI2Rs(theta), RI2Rs(tmp));
                fflush(fout);
            }
#endif

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
            consigne = SIGN(consigne)*(SQR(consigne)>>2)/abs(tmp);
        }

        // update set point of each motor
        consigne_l = (((long long)_mul_l*(long long)consigne)>>SHIFT) - tmp; // (IpP<<SHIFT)
        consigne_r = (((long long)_mul_r*(long long)consigne)>>SHIFT) + tmp;

#if defined(ARCH_X86_LINUX) && defined(ASSERV_LOGS)
        if(fout){
            long long tmp2;

            if(curr_traj_step&1){
                tmp2 = SQRT((SQR(x - traj[curr_traj][curr_traj_step>>1].c_x) + SQR(y - traj[curr_traj][curr_traj_step>>1].c_y))>>SHIFT) - abs(traj[curr_traj][curr_traj_step>>1].c_r);
                if(traj[curr_traj][curr_traj_step>>1].c_r > 0){
                    tmp2*=-1;
                }
            }
            else{
                long long a, b, c;
                a = traj[curr_traj][curr_traj_step>>1].p2_y - traj[curr_traj][curr_traj_step>>1].p1_y;
                b = traj[curr_traj][curr_traj_step>>1].p1_x - traj[curr_traj][curr_traj_step>>1].p2_x;
                c = (-a*traj[curr_traj][curr_traj_step>>1].p1_x -b*traj[curr_traj][curr_traj_step>>1].p1_y)>>SHIFT;

                tmp2 = (a*x + b*y + (c<<SHIFT))/SQRT((SQR(a) + SQR(b))>>SHIFT);
            }

            fprintf(fout, "%u,%i,%lli,%i,%i,%i,%lli,%i,%i,%i\n", millis(), curr_traj_step&1, tmp, _mul_l, _mul_r, consigne, tmp2, x, y, theta);
            fflush(fout);
        }
#endif
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
