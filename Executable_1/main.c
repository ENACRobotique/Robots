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

// trajectory definitions
struct {
    int p1_x;
    int p1_y;
    int p2_x;
    int p2_y;
    int c_x;
    int c_y;
    int r;
} traj[] = {
#if 0
  {isD2I(30.00), isD2I(100.00), isD2I(83.56), isD2I(64.91), isD2I(100.00), isD2I(90.00), isD2I(30.00)},
  {isD2I(100.00), isD2I(60.00), isD2I(180.00), isD2I(60.00), isD2I(180.00), isD2I(30.00), isD2I(30.00)},
  {isD2I(192.86), isD2I(57.11), isD2I(250.00), isD2I(30.00), isD2I(250.00), isD2I(30.00), isD2I(0.00)}
#elif 0
  {isD2I(30.00), isD2I(100.00), isD2I(83.56), isD2I(64.91), isD2I(100.00), isD2I(90.00), isD2I(30.00)},
  {isD2I(119.00), isD2I(66.79), isD2I(227.33), isD2I(155.48), isD2I(240.00), isD2I(140.00), isD2I(20.00)},
  {isD2I(247.86), isD2I(121.61), isD2I(168.22), isD2I(87.59), isD2I(180.00), isD2I(60.00), isD2I(30.00)},
  {isD2I(180.00), isD2I(30.00), isD2I(250.00), isD2I(30.00), isD2I(250.00), isD2I(30.00), isD2I(0.00)}
#elif 0
// bord mur evit verres
  {isD2I(30.00), isD2I(100.00), isD2I(70.21), isD2I(41.42), isD2I(90.00), isD2I(55.00), isD2I(24.00)},
  {isD2I(90.00), isD2I(31.00), isD2I(120.00), isD2I(31.00), isD2I(120.00), isD2I(55.00), isD2I(24.00)},
  {isD2I(120.00), isD2I(31.00), isD2I(180.00), isD2I(31.00), isD2I(180.00), isD2I(55.00), isD2I(24.00)},
  {isD2I(180.00), isD2I(31.00), isD2I(210.00), isD2I(31.00), isD2I(210.00), isD2I(55.00), isD2I(24.00)},
  {isD2I(221.79), isD2I(34.09), isD2I(250.00), isD2I(50.00), isD2I(250.00), isD2I(50.00), isD2I(0.00)}
#else
// zigzag verres
  {isD2I(30.00), isD2I(100.00), isD2I(78.64), isD2I(126.14), isD2I(90.00), isD2I(105.00), isD2I(24.00)},
  {isD2I(113.77), isD2I(101.71), isD2I(111.23), isD2I(83.29), isD2I(135.00), isD2I(80.00), isD2I(24.00)},
  {isD2I(158.77), isD2I(83.29), isD2I(156.23), isD2I(101.71), isD2I(180.00), isD2I(105.00), isD2I(24.00)},
  {isD2I(201.85), isD2I(95.07), isD2I(188.15), isD2I(64.93), isD2I(210.00), isD2I(55.00), isD2I(24.00)},
  {isD2I(210.00), isD2I(79.00), isD2I(180.00), isD2I(79.00), isD2I(180.00), isD2I(55.00), isD2I(24.00)},
  {isD2I(160.80), isD2I(69.40), isD2I(139.20), isD2I(40.60), isD2I(120.00), isD2I(55.00), isD2I(24.00)},
  {isD2I(98.15), isD2I(64.93), isD2I(111.85), isD2I(95.07), isD2I(90.00), isD2I(105.00), isD2I(24.00)},
  {isD2I(78.64), isD2I(126.14), isD2I(30.00), isD2I(100.00), isD2I(30.00), isD2I(100.00), isD2I(0.00)}
#endif
};
int step_traj = 0, step_rotdir;

enum {
  S_WAIT,
  S_RUN_TRAJ
} state = S_RUN_TRAJ;

void mybreak_i() {
  static int i = 0;
  i++;
}

int nb_l = 0, nb_c = 0;
int m_l, m_c;
unsigned int start_us;

int main(void) {
  unsigned int prevAsserv=0, prevLed=0;
  int ticks_l, ticks_r;
  int consigne_l = 0, _consigne_l;
  int consigne_r = 0, _consigne_r;
  int consigne = isRpS2IpP(1);
  long long tmp;

  int x, y, theta, ct, st, n_x, n_y;
  int gx, gy; // goal
  int v, alpha;
  unsigned long long sqdist, prev_sqdist = 0;

  x = traj[0].p1_x; // (I<<SHIFT)
  y = traj[0].p1_y;
  theta = iROUND(0.*PI/180.*D2I(RDIAM)*dSHIFT);

  gx = traj[0].p2_x;
  gy = traj[0].p2_y;
  _consigne_l = consigne;
  _consigne_r = consigne;

  gpio_init_all();  // use fast GPIOs

  pwm_init(0, 1024);  // 29.3kHz update rate => not audible

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
      if(theta > isRPI) {
        theta -= (isRPI<<1);
      }
      else if(theta < -isRPI) {
        theta += (isRPI<<1);
      }
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
        if(sqdist < lsROUND(D2I(1.5)*D2I(1.5))) { // we are near the goal
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
                _consigne_l = (long long)consigne*(traj[step_traj>>1].r + isD2I(RDIAM/2.))/traj[step_traj>>1].r;
                _consigne_r = (long long)consigne*(traj[step_traj>>1].r - isD2I(RDIAM/2.))/traj[step_traj>>1].r;
              }
              else {  // counterclockwise
                // set speed a priori
                _consigne_l = (long long)consigne*(traj[step_traj>>1].r - isD2I(RDIAM/2.))/traj[step_traj>>1].r;
                _consigne_r = (long long)consigne*(traj[step_traj>>1].r + isD2I(RDIAM/2.))/traj[step_traj>>1].r;
              }
            }
            else {
              gx = traj[step_traj>>1].p2_x;
              gy = traj[step_traj>>1].p2_y;

              _consigne_l = consigne; // set speed a priori
              _consigne_r = consigne; // set speed a priori
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
        consigne_l = _consigne_l - tmp; // (IpP<<SHIFT)
        consigne_r = _consigne_r + tmp;
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
