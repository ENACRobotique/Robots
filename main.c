#include <targets/LPC2000.h>
#include <ctl_api.h>
#include <math.h>
#include <limits.h>

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

#define SQR(v) ((long long)(v)*(v))

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

typedef struct {
// segment
    int p1_x;
    int p1_y;
    int p2_x;
    int p2_y;
    int seg_len;  // length of the segment trajectory in increments
// circle
    int c_x;
    int c_y;
    int r;
    int arc_len;  // length of the circle trajectory in increments
    int arc_sp_max; // max_speed on the circle
} sTrajEl_t;
sTrajEl_t *traj = NULL;
int step_traj = 0, step_rotdir, num_steps;


int x, y, theta;
int gx, gy; // goal
int d_consigne = isRpS2IpP(0.5);
int _mul_l = 1<<SHIFT, _mul_r = 1<<SHIFT;
enum {
  S_WAIT,
  S_RUN_TRAJ // TODO S_INIT_TRAJ to consume less cpu resource in i2c handler
} state = S_WAIT;

void run_traj(sTrajEl_t *t, int nb, int angle) {
  int i;

  traj = t;
  step_traj = 0;
  num_steps = nb;

  for(i=0; i<nb; i++)
    traj[i].arc_sp_max = SQRT(1.5*1.5*(double)traj[i].r);

  x = traj[0].p1_x; // (I<<SHIFT)
  y = traj[0].p1_y;
  theta = angle;

  // speed
  d_consigne = isRpS2IpP(1);
  _mul_l = 1<<SHIFT;
  _mul_r = 1<<SHIFT;

  // current goal
  gx = traj[0].p2_x;
  gy = traj[0].p2_y;

  state = S_RUN_TRAJ;
}

sTrajEl_t traj_blue[] = {
  {isD2I(7.50), isD2I(100.00), isD2I(88.75), isD2I(112.90), isD2I(82.26), isD2I(90.00), isD2I(105.00), isD2I(8.00), isD2I(16.43)},
  {isD2I(97.58), isD2I(102.44), isD2I(82.42), isD2I(57.56), isD2I(47.37), isD2I(90.00), isD2I(55.00), isD2I(8.00), isD2I(38.89)},
  {isD2I(93.66), isD2I(62.11), isD2I(20.00), isD2I(100.00), isD2I(82.83), isD2I(20.00), isD2I(100.00), isD2I(0.00), isD2I(0.00)},
/*  {isD2I(7.50), isD2I(100.00), isD2I(88.75), isD2I(112.90), isD2I(82.26), isD2I(90.00), isD2I(105.00), isD2I(8.00), isD2I(13.83), 0},
  {isD2I(98.00), isD2I(105.00), isD2I(98.00), isD2I(55.00), isD2I(50.00), isD2I(90.00), isD2I(55.00), isD2I(8.00), isD2I(17.91), 0},
  {isD2I(85.05), isD2I(48.72), isD2I(20.00), isD2I(100.00), isD2I(82.83), isD2I(20.00), isD2I(100.00), isD2I(0.00), isD2I(0.00), 0}*/
};
sTrajEl_t traj_red[] = {
  {isD2I(292.50), isD2I(100.00), isD2I(211.25), isD2I(112.90), isD2I(82.26), isD2I(210.00), isD2I(105.00), isD2I(8.00), isD2I(13.83), 0},
  {isD2I(202.00), isD2I(105.00), isD2I(202.00), isD2I(55.00), isD2I(50.00), isD2I(210.00), isD2I(55.00), isD2I(8.00), isD2I(17.91), 0},
  {isD2I(214.95), isD2I(48.72), isD2I(280.00), isD2I(100.00), isD2I(82.83), isD2I(280.00), isD2I(100.00), isD2I(0.00), isD2I(0.00), 0}
};

void i2chnd(struct i2c_transaction *t, void *userp) {
  if(t->len_r) {
    switch(t->buf[0]) {
    case 1: // go blue
      run_traj(traj_blue, sizeof(traj_blue)/sizeof(*traj_blue), iROUND(0.*PI/180.*D2I(RDIAM)*dSHIFT));

      break;
    case 2: // go red
      run_traj(traj_red, sizeof(traj_red)/sizeof(*traj_red), iROUND(180.*PI/180.*D2I(RDIAM)*dSHIFT));

      break;
    case 3: // set desired speed
      d_consigne = (t->buf[1]*isRpS2IpP(1.5))>>8;

      break;
    default:
      break;
    }
  }
}

// stats
int nb_l = 0, nb_c = 0;
int m_l, m_c;
unsigned int start_us;
// DEBUG
void mybreak_i() {
  static int i = 0;
  i++;
}

int main(void) {
  unsigned int prevAsserv=0, prevLed=0;
  int ticks_l, ticks_r;
  int consigne = d_consigne;
  int consigne_l = 0;
  int consigne_r = 0;
  long long tmp, dtime, dist_lim;

  int ct, st, n_x, n_y, i;
  int v, alpha;
  unsigned long long sqdist, prev_sqdist = 0, dist;

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
      if(millis() - prevAsserv > 30) {
        // for debugger
        prevAsserv = millis();
        ctl_global_interrupts_disable();  // prevent _ticks_* to be modified by an interruption caused by an increment
        _ticks_l = 0; // (IpP<<SHIFT)
        _ticks_r = 0; // (IpP<<SHIFT)
        ctl_global_interrupts_enable();
        continue;
      }
      prevAsserv += 20;

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
      case S_RUN_TRAJ:
        // squared distance to the next goal
        sqdist = (SQR(gy - y) + SQR(gx - x))>>SHIFT;

        //consigne = isRpS2IpP(0.5);
//        dist_lim = ((long long)SQR(v)*5/(long long)AMAX)>>(SHIFT+2);
        dist_lim = ((long long)SQR(v)*8/(long long)AMAX)>>(SHIFT+1);

        if(step_traj&1) { // portion de cercle
          consigne = min(traj[step_traj>>1].arc_sp_max, d_consigne);
          // TODO
        }
        else { // portion de segment
          consigne = d_consigne;
          int dbg_consigne;
          for(i = step_traj>>1, dist = SQRT(sqdist); i < num_steps && dist < dist_lim; ++i, dist += traj[i].seg_len + traj[i-1].arc_len) {
            dtime = abs((((long long)v - (long long)traj[i].arc_sp_max)*8/(long long)AMAX)>>1);
            dbg_consigne = (int)( ((long long)dist<<SHIFT)/(long long)dtime );
            if(dbg_consigne < consigne)
              consigne = dbg_consigne;
          }
        }

        if(sqdist < lsROUND(D2I(1)*D2I(1))) { // we are near the goal
          if(!(step_traj&1) && traj[step_traj>>1].r < isD2I(1)) { // no next step
            step_traj = 0;
            consigne_l = 0;
            consigne_r = 0;
            state = S_WAIT;
            continue;
          }
          else {
            step_traj++;

            if(step_traj&1) {
              gx = traj[(step_traj>>1) + 1].p1_x;
              gy = traj[(step_traj>>1) + 1].p1_y;
              step_rotdir = ((long long)traj[step_traj>>1].p1_x - traj[step_traj>>1].p2_x)*((long long)traj[step_traj>>1].c_y - traj[step_traj>>1].p2_y) - ((long long)traj[step_traj>>1].p1_y - traj[step_traj>>1].p2_y)*((long long)traj[step_traj>>1].c_x - traj[step_traj>>1].p2_x) > 0;
              if(step_rotdir) { // clockwise
                // set speed a priori
                _mul_l = ((long long)(traj[step_traj>>1].r + isD2I(RDIAM/2.))<<SHIFT)/traj[step_traj>>1].r;
                _mul_r = ((long long)(traj[step_traj>>1].r - isD2I(RDIAM/2.))<<SHIFT)/traj[step_traj>>1].r;
              }
              else {  // counterclockwise
                // set speed a priori
                _mul_l = ((long long)(traj[step_traj>>1].r - isD2I(RDIAM/2.))<<SHIFT)/traj[step_traj>>1].r;
                _mul_r = ((long long)(traj[step_traj>>1].r + isD2I(RDIAM/2.))<<SHIFT)/traj[step_traj>>1].r;
              }
            }
            else {
              gx = traj[step_traj>>1].p2_x;
              gy = traj[step_traj>>1].p2_y;

              // set speed a priori
              _mul_l = 1<<SHIFT;
              _mul_r = 1<<SHIFT;
            }
          }

          //sqdist = (SQR(gy - y) + SQR(gx - x))>>SHIFT;
        }

        if(step_traj&1) { // circle
          // future error
          n_x = x + ((long long)(v*ct)>>(SHIFT-2)); // position prediction in 4 periods
          n_y = y + ((long long)(v*st)>>(SHIFT-2));
          if(step_rotdir) { // clockwise
            tmp = -( (SQR(n_x - traj[step_traj>>1].c_x) + SQR(n_y - traj[step_traj>>1].c_y))/traj[step_traj>>1].r - traj[step_traj>>1].r );
          }
          else {
            tmp = ( (SQR(n_x - traj[step_traj>>1].c_x) + SQR(n_y - traj[step_traj>>1].c_y))/traj[step_traj>>1].r - traj[step_traj>>1].r );
          }

          tmp>>=1;  // gain

          // time stats... result 160µs!
          m_c = (m_c*nb_c + ((micros() - start_us)<<8))/(nb_c+1);
          nb_c++;
        }
        else {  // straight line
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
