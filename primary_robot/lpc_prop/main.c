#include <lpc214x.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include <gpio.h>
#include <eint.h>
#include <ime.h>
#include <pwm.h>
#include <sys_time.h>
#include <trigo.h>
#include "shared/botNet_core.h"
#include "shared/bn_debug.h"

#include "params.h"
#include "controller.h"

// compile-time config
//#define ASSERV_TEST
//#define NETWORK_TEST
//#define TIME_STATS

#if defined(NETWORK_TEST) && !defined(ASSERV_TEST)
#warning "NETWORK_TEST is useless when ASSERV_TEST is not defined"
#endif

#define SQR(v) ((long long)(v)*(v))

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

#ifndef ASSERV_TEST
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
#endif

int x, y, theta; // robot position (I<<SHIFT), robot heading (I.rad<<SHIFT)
#ifndef ASSERV_TEST
int gx, gy; // goal (I<<SHIFT)
int d_consigne = isDpS2IpP(15. /* cm/s */); // desired speed (IpP<<SHIFT)
int _mul_l, _mul_r; // speed multiplier for each wheel (<<SHIFT)
enum {
    S_WAIT, // no action asked (we are stopped)
    S_CHG_TRAJ, // new trajectory to follow
    S_RUN_TRAJ // we are following a trajectory
} state = S_WAIT; // state of the trajectory follow
#endif

#ifdef TIME_STATS
// stats
#ifndef ASSERV_TEST
int nb_l = 0, nb_c = 0;
int m_l, m_c;
#endif
int nb_loop;
int m_loop;
unsigned int start_us;
#endif

int main(void) {
    unsigned int prevAsserv=0, prevLed=0;
    int ticks_l, ticks_r;
#ifdef ASSERV_TEST
    int consigne_l = isDpS2IpP(15. /* cm/s */); // desired speed (IpP<<SHIFT)
    int consigne_r = isDpS2IpP(15. /* cm/s */); // desired speed (IpP<<SHIFT)
#else
    int consigne_l = 0;
    int consigne_r = 0;
    unsigned int prevPos=0;
#endif

    int ct, st;
    int v;

#ifndef ASSERV_TEST
    int consigne;
    long long tmp, dtime;
    int n_x, n_y, i;
    int alpha, dist, dist_tmp, dist_lim;
#endif
#if !defined(ASSERV_TEST) || (defined(ASSERV_TEST) && defined(NETWORK_TEST))
    sMsg msg;
#endif

    gpio_init_all();  // use fast GPIOs

    pwm_init(0, 1024);  // 29.3kHz update rate => not audible

#if !defined(ASSERV_TEST) || (defined(ASSERV_TEST) && defined(NETWORK_TEST))
    bn_init();
#endif

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

    // sortie moteur
    motor_controller_init();

    // init time management
    sys_time_init();

    global_IRQ_enable();

#ifndef ASSERV_TEST
    bn_printDbg("start prop");
#endif

    // main loop
    while(1) {
        sys_time_update();

#if defined(ASSERV_TEST) && defined(NETWORK_TEST)
        if(bn_receive(&msg) > 0) {
            switch(msg.header.type){
            case E_DATA: // beacon
                gpio_write(0, 31, !msg.payload.raw[0]);

                break;
            default:
                break;
            }
        }
#endif

#ifndef ASSERV_TEST
        if(bn_receive(&msg) > 0) {
            switch(msg.header.type){
            case E_DATA: // beacon
                gpio_write(0, 31, msg.payload.raw[0]);

                //        sb_printfDbg(ADDRX_REMOTE_IA, "got led flash (%i)", msg.payload.raw[0]);

                break;
            case E_TRAJ:{
                sTrajElRaw_t *te = &msg.payload.traj;
                int error = 0;

                //        sb_printfDbg(ADDRX_REMOTE_IA, "got traj step (t%hu, s%hu)", te->tid, te->sid);

                if( curr_traj_insert_sid > 0 && te->tid == curr_tid ) { // we are currently following a trajectory and we got some new steps to add to this trajectory
                    if( curr_traj_insert_sid < TRAJ_MAX_SIZE ) {
                        if( te->sid == curr_traj_insert_sid ) {
                            memcpy(&traj[curr_traj][curr_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
                            traj[curr_traj][curr_traj_insert_sid].is_ok = 0;
                            curr_traj_insert_sid++;

                            state = S_RUN_TRAJ;
                        }
                        else {
                            error = 1; // TODO error: bad step => invalidate all trajectory and ask new one
                        }
                    }
                    else {
                        error = 2; // TODO error: too much trajectory steps received
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
                            error = 3; // TODO error: bad step => invalidate all trajectory and ask new one
                        }
                    }
                    else {
                        error = 4; // TODO error: too much trajectory steps received
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
                        error = 5; // TODO error: bad step => invalidate all trajectory and ask new one
                    }
                }

                //        sb_printDbg(ADDRX_REMOTE_IA, "got traj step, error=", error, 0);

                if( error ) {
                    // error...
                }
                break;
            }
            case E_POS:
                if(msg.payload.pos.id == 0) { // prim robot
                    x = isD2I(msg.payload.pos.x); // (I<<SHIFT)
                    y = isD2I(msg.payload.pos.y); // (I<<SHIFT)
                    theta = isROUND( D2I(RDIAM)*msg.payload.pos.theta ); // (rad.I<<SHIFT)

                    bn_printDbg("got pos");
                }
                break;
                //case 3: // set desired speed
                //d_consigne = isDpS2IpP(*(float *)(&t->buf[4]));
                //break;
                // TODO case get position (send position periodically with uncertainty to the fixed beacons for better position estimation)

            default:
                bn_printDbg("got unhandled msg");
                break;
            }
        }
#endif

        if(millis() - prevAsserv >= 20) { // each 20 milliseconds
            prevAsserv += 20;
            // TODO reduce period, may use interrupt on level change to double number of irqs
            if(millis() - prevAsserv > 10) { // we are very late, do not take care of these data
                // for debugger
                prevAsserv = millis();
                global_IRQ_disable();  // prevent _ticks_* to be modified by an interruption caused by an increment
                _ticks_l = 0; // (IpP<<SHIFT)
                _ticks_r = 0; // (IpP<<SHIFT)
                global_IRQ_enable();
                continue;
            }

#ifdef TIME_STATS
            start_us = micros();
#endif

            // get current number of ticks per sampling period
            global_IRQ_disable();  // prevent _ticks_? to be modified by an interruption caused by an increment
            ticks_l = _ticks_l<<SHIFT; _ticks_l = 0; // (IpP<<SHIFT)
            ticks_r = _ticks_r<<SHIFT; _ticks_r = 0; // (IpP<<SHIFT)
            global_IRQ_enable();

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

#ifndef ASSERV_TEST
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
                        if(dtime > 0){
                            consigne = MIN(consigne, (int)( ((long long)dist_tmp<<SHIFT)/(long long)dtime ));
                        }
                    }
                }

                if(dist < isD2I(1)) { // we are near the goal
                    if(!(curr_traj_step&1) && abs(traj[curr_traj][curr_traj_step>>1].c_r) < isD2I(1)) { // no next step
                        consigne_l = 0;
                        consigne_r = 0;
                        state = S_WAIT;
                        continue;
                    }
                    else {
                        if( ((curr_traj_step+1)>>1) >= curr_traj_insert_sid ){ // we do not have any steps...
                            consigne_l = 0;
                            consigne_r = 0;
                            continue;
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

                    tmp>>=1;  // gain

#ifdef TIME_STATS
                    // time stats... result 160µs! (outdated)
                    m_c = (m_c*nb_c + ((micros() - start_us)<<8))/(nb_c+1);
                    nb_c++;
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
                    m_l = (m_l*nb_l + ((micros() - start_us)<<8))/(nb_l+1);
                    nb_l++;
#endif
                }

                //        if(abs(tmp) > consigne) { // very high compensation term
                //          mybreak_i();
                //        }

                // update set point of each motor
                consigne_l = (((long long)_mul_l*(long long)consigne)>>SHIFT) - tmp; // (IpP<<SHIFT)
                consigne_r = (((long long)_mul_r*(long long)consigne)>>SHIFT) + tmp;
                break;
            }
#endif

#ifdef TIME_STATS
            // time stats... result 22µs! (in ASSERV_TEST)
            m_loop = (m_loop*nb_loop + ((micros() - start_us)<<8))/(nb_loop+1);
            nb_loop++;
#endif
        }

        if(millis() - prevPos >= 100) {
            prevPos = millis();

            msg.header.destAddr = ADDRD_MAIN_PROP_SIMU;
            msg.header.type = E_POS;
            msg.header.size = sizeof(msg.payload.pos);
            msg.payload.pos.id = 0; // main robot
            msg.payload.pos.x = I2Ds(x);
            msg.payload.pos.y = I2Ds(y);
            msg.payload.pos.theta = RI2Rs(theta);

            bn_send(&msg);
        }

        if(millis() - prevLed >= 250) {
            prevLed += 250;
            gpio_toggle(1, 24); // LED flashing
        }
    }

    return 0;
}
