#include <targets/LPC2000.h>
#include <ctl_api.h>
#include <math.h>
#include <limits.h>
#include <string.h>

#include "gpio.h"
#include "eint.h"
#include "pwm.h"
#include "controller.h"
#include "sys_time.h"
#include "params.h"
#include "trigo.h"
#include "i2c0.h"

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

#ifndef min
#define min(a, b) ((a)>(b)?(b):(a))
#endif
#ifndef max
#define max(a, b) ((a)>(b)?(a):(b))
#endif
#ifndef MINMAX
#define MINMAX(m, v, M) max(m, min(v, M))
#endif
//#ifndef SWAP32
//#define SWAP32(a, b) do { uint32_t c; c = (a); (a) = (b); (b) = c; } while(0)
//#endif

#define SQR(v) ((long long)(v)*(v))

// DEBUG
void mybreak_i() {
  static int i = 0;
  i++;
}

volatile int _ticks_l=0, _ticks_r=0;

void _isr_left() { // irq17
  if(gpio_read(1, 16))
    _ticks_l--;
  else
    _ticks_l++;

  EXTINT = BIT(3);
}

void _isr_right() { // irq14
  if(gpio_read(1, 17))
    _ticks_r++;
  else
    _ticks_r--;

  EXTINT = BIT(0);
}

#define TRAJ_MAX_SIZE (16)

typedef struct {
// segment
  float p1_x;
  float p1_y;
  float p2_x;
  float p2_y;
  float seg_len;
// circle
  float c_x;
  float c_y;
  float c_r;
  float arc_len;
// trajectory data
  uint16_t tid; // trajectory identifier
  uint16_t sid; // step identifier
} sTrajElRaw_t;

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
volatile sTrajEl_t traj[2][TRAJ_MAX_SIZE];
volatile int curr_traj = 0; // current followed trajectory (0:1)
int curr_traj_step; // current step of the current trajectory (0:curr_traj_insert_sid*2-1)
volatile int curr_traj_insert_sid = 0; // index at which data will be inserted in the current trajectory
volatile int next_traj_insert_sid = 0; // index at which data will be inserted in the next trajectory
volatile uint16_t curr_tid, next_tid; // current and next trajectory identifiers

void traj_copy(volatile sTrajElRaw_t *dst, volatile sTrajElRaw_t *src) {
  dst->p1_x = src->p1_x;
  dst->p1_y = src->p1_y;
  dst->p2_x = src->p2_x;
  dst->p2_y = src->p2_y;
  dst->seg_len = src->seg_len;
  dst->c_x = src->c_x;
  dst->c_y = src->c_y;
  dst->c_r = src->c_r;
  dst->arc_len = src->arc_len;
  dst->tid = src->tid;
  dst->sid = src->sid;
}

void traj_conv(volatile sTrajEl_t *t) {
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

    t->arc_sp_max = SQRT(1.5*1.5*(double)t->c_r);
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

//#define TRAJ_DEBUG
#ifdef TRAJ_DEBUG
unsigned int i_debug_tab_traj = 0;
struct {
  unsigned long t;
  sTrajElRaw_t traj;
  int error;
  int len_r;
} debug_tab_traj[16];
#endif

void i2chnd(struct i2c_transaction *t, void *userp) {
  int error = 0;

  if(t->len_r) {
    switch(t->buf[0]) {
    case 1:{ // new trajectory step received
      sTrajElRaw_t *te = (sTrajElRaw_t *)(&t->buf[4]);

#ifdef TRAJ_DEBUG
      if(i_debug_tab_traj < sizeof(debug_tab_traj)/sizeof(*debug_tab_traj)) {
        debug_tab_traj[i_debug_tab_traj].t = millis();
        debug_tab_traj[i_debug_tab_traj].len_r = t->len_r;
        traj_copy(&debug_tab_traj[i_debug_tab_traj].traj, te);
      }
#endif

      if( curr_traj_insert_sid > 0 && te->tid == curr_tid ) { // we are currently following a trajectory and we got some new steps to add to this trajectory
        if( curr_traj_insert_sid < TRAJ_MAX_SIZE ) {
          if( te->sid == curr_traj_insert_sid ) {
            traj_copy(&traj[curr_traj][curr_traj_insert_sid].raw, te);
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
      else if( next_traj_insert_sid > 0 && te->tid == next_tid ) { // we already got some new steps but we stil didn't switch to those (next_tid is valid)
        if( next_traj_insert_sid < TRAJ_MAX_SIZE ) {
          if( te->sid == next_traj_insert_sid ) {
            traj_copy(&traj[!curr_traj][next_traj_insert_sid].raw, te);
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
          traj_copy(&traj[!curr_traj][next_traj_insert_sid].raw, te);
          traj[!curr_traj][next_traj_insert_sid].is_ok = 0;
          next_traj_insert_sid++;

          state = S_CHG_TRAJ;
        }
        else {
          error = 5; // TODO error: bad step => invalidate all trajectory and ask new one
        }
      }

      if( t->len_r != sizeof(sTrajElRaw_t) + 4 ) {
        error |= BIT(7);
      }

      if( error ) {
        mybreak_i();
      }

#ifdef TRAJ_DEBUG
      if(i_debug_tab_traj < sizeof(debug_tab_traj)/sizeof(*debug_tab_traj)) {
        debug_tab_traj[i_debug_tab_traj].error = error;
        i_debug_tab_traj++;
      }
#endif

      break;
    }

    case 3: // set desired speed
      d_consigne = isDpS2IpP(*(float *)(&t->buf[4]));
      break;

    case 4: // set position
      x = isD2I(*(float *)(&t->buf[4])); // (I<<SHIFT)
      y = isD2I(*(float *)(&t->buf[8])); // (I<<SHIFT)
      theta = isROUND( D2I(RDIAM)*(*(float *)(&t->buf[12])) ); // (rad.I<<SHIFT)
      break;

    // TODO case get position (send position periodically with uncertainty to the fixed beacons for better position estimation)

    default:
      break;
    }
  }
}

// stats
int nb_l = 0, nb_c = 0;
int m_l, m_c;
unsigned int start_us;

int main(void) {
  unsigned int prevAsserv=0, prevLed=0;
  int ticks_l, ticks_r;
  int consigne;
  int consigne_l = 0;
  int consigne_r = 0;
  long long tmp, dtime;

  int ct, st, n_x, n_y, i;
  int v, alpha, dist, dist_tmp, dist_lim;

  int step_rotdir;

  gpio_init_all();  // use fast GPIOs

  pwm_init(0, 1024);  // 29.3kHz update rate => not audible

  i2c0_init(400000, 0x44, i2chnd, NULL);

// sortie LED
  gpio_output(1, 24);
  gpio_write(1, 24, 0); // LED on

// entrées quadrature
  gpio_input(1, 16);  // gauche
  gpio_input(1, 17);  // droit

// interruptions codeurs
  eint_init(_isr_right, _isr_left);

// sortie moteur
  motor_controller_init();

// init time management
  sys_time_init();

  ctl_global_interrupts_enable();

// main loop
  while(1) {
    sys_time_update();

    if(millis() - prevAsserv >= 20) { // each 20 milliseconds
      prevAsserv = millis();
      // TODO reduce period
      if(millis() - prevAsserv > 10) { // we are very late, do not take care of these data
        // for debugger
        prevAsserv = millis();
        ctl_global_interrupts_disable();  // prevent _ticks_* to be modified by an interruption caused by an increment
        _ticks_l = 0; // (IpP<<SHIFT)
        _ticks_r = 0; // (IpP<<SHIFT)
        ctl_global_interrupts_enable();
        continue;
      }

      start_us = micros();

      // get current number of ticks per sampling period
      ctl_global_interrupts_disable();  // prevent _ticks_? to be modified by an interruption caused by an increment
      ticks_l = _ticks_l<<SHIFT; _ticks_l = 0; // (IpP<<SHIFT)
      ticks_r = _ticks_r<<SHIFT; _ticks_r = 0; // (IpP<<SHIFT)
      ctl_global_interrupts_enable();

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
        ctl_global_interrupts_disable(); // critical section

          // current trajectory
          curr_traj = !curr_traj;
          curr_tid = next_tid;
          curr_traj_insert_sid = next_traj_insert_sid;
          next_traj_insert_sid = 0;
          // current state
          state = S_RUN_TRAJ;

        ctl_global_interrupts_enable();

        curr_traj_step = 0;

        // speed
        _mul_l = 1<<SHIFT;
        _mul_r = 1<<SHIFT;

        traj_conv(&traj[curr_traj][curr_traj_step>>1]);

        // current goal
        gx = traj[curr_traj][curr_traj_step>>1].p2_x;
        gy = traj[curr_traj][curr_traj_step>>1].p2_y;

//        break;        fall through to save 20 ms

      case S_RUN_TRAJ:
        // distance to the next goal
        dist = SQRT( (SQR(gy - y) + SQR(gx - x))>>SHIFT );

        // distance needed to stop (knowing the current speed and with a margin of a factor 3!)
        dist_lim = (int)( ((long long)SQR(v)*3/(long long)AMAX)>>SHIFT );

        if(curr_traj_step&1) { // portion de cercle
          consigne = min(traj[curr_traj][curr_traj_step>>1].arc_sp_max, d_consigne);
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
            dtime = abs(((long long)v - (long long)traj[curr_traj][i].arc_sp_max)*3/(long long)AMAX);
            consigne = min(consigne, (int)( ((long long)dist_tmp<<SHIFT)/(long long)dtime ));
          }
        }

        if(dist < isD2I(1)) { // we are near the goal
          if(!(curr_traj_step&1) && traj[curr_traj][curr_traj_step>>1].c_r < isD2I(1)) { // no next step
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
              gx = traj[curr_traj][(curr_traj_step>>1) + 1].p1_x;
              gy = traj[curr_traj][(curr_traj_step>>1) + 1].p1_y;
              step_rotdir = ((long long)traj[curr_traj][curr_traj_step>>1].p1_x - traj[curr_traj][curr_traj_step>>1].p2_x)*((long long)traj[curr_traj][curr_traj_step>>1].c_y - traj[curr_traj][curr_traj_step>>1].p2_y) - ((long long)traj[curr_traj][curr_traj_step>>1].p1_y - traj[curr_traj][curr_traj_step>>1].p2_y)*((long long)traj[curr_traj][curr_traj_step>>1].c_x - traj[curr_traj][curr_traj_step>>1].p2_x) > 0;
              if(step_rotdir) { // clockwise
                // set speed a priori
                _mul_l = ((long long)(traj[curr_traj][curr_traj_step>>1].c_r + isD2I(RDIAM/2.))<<SHIFT)/traj[curr_traj][curr_traj_step>>1].c_r;
                _mul_r = ((long long)(traj[curr_traj][curr_traj_step>>1].c_r - isD2I(RDIAM/2.))<<SHIFT)/traj[curr_traj][curr_traj_step>>1].c_r;
              }
              else {  // counterclockwise
                // set speed a priori
                _mul_l = ((long long)(traj[curr_traj][curr_traj_step>>1].c_r - isD2I(RDIAM/2.))<<SHIFT)/traj[curr_traj][curr_traj_step>>1].c_r;
                _mul_r = ((long long)(traj[curr_traj][curr_traj_step>>1].c_r + isD2I(RDIAM/2.))<<SHIFT)/traj[curr_traj][curr_traj_step>>1].c_r;
              }
            }
            else {
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
          if(step_rotdir) { // clockwise
            tmp = -( (SQR(n_x - traj[curr_traj][curr_traj_step>>1].c_x) + SQR(n_y - traj[curr_traj][curr_traj_step>>1].c_y))/traj[curr_traj][curr_traj_step>>1].c_r - traj[curr_traj][curr_traj_step>>1].c_r );
          }
          else {
            tmp =  ( (SQR(n_x - traj[curr_traj][curr_traj_step>>1].c_x) + SQR(n_y - traj[curr_traj][curr_traj_step>>1].c_y))/traj[curr_traj][curr_traj_step>>1].c_r - traj[curr_traj][curr_traj_step>>1].c_r );
          }

          tmp>>=1;  // gain

          // time stats... result 160µs!
          m_c = (m_c*nb_c + ((micros() - start_us)<<8))/(nb_c+1);
          nb_c++;
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

          // time stats... result 257µs!
          m_l = (m_l*nb_l + ((micros() - start_us)<<8))/(nb_l+1);
          nb_l++;
        }

        if(abs(tmp) > consigne) { // very high compensation term
          mybreak_i();
        }

        // update set point of each motor
        consigne_l = (((long long)_mul_l*(long long)consigne)>>SHIFT) - tmp; // (IpP<<SHIFT)
        consigne_r = (((long long)_mul_r*(long long)consigne)>>SHIFT) + tmp;
        break;
      }
    }

    if(millis() - prevLed >= 250) {
      prevLed += 250;
      gpio_toggle(1, 24); // LED flashing
    }
  }

  return 0;
}
