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
#else
  {isD2I(30.00), isD2I(100.00), isD2I(83.56), isD2I(64.91), isD2I(100.00), isD2I(90.00), isD2I(30.00)},
  {isD2I(119.00), isD2I(66.79), isD2I(227.33), isD2I(155.48), isD2I(240.00), isD2I(140.00), isD2I(20.00)},
  {isD2I(247.86), isD2I(121.61), isD2I(168.22), isD2I(87.59), isD2I(180.00), isD2I(60.00), isD2I(30.00)},
  {isD2I(180.00), isD2I(30.00), isD2I(250.00), isD2I(30.00), isD2I(250.00), isD2I(30.00), isD2I(0.00)}
#endif
};
int step_traj = 0;

int main(void) {
  unsigned int prevAsserv=0, prevLed=0;
  int ticks_l, ticks_r;
  int consigne_l = 0, _consigne_l;
  int consigne_r = 0, _consigne_r;
  int consigne = isRpS2IpP(0.75);
  long long tmp;

  int x, y, theta;
  int gx, gy; // goal
  int v, alpha;
  unsigned long long sqdist;

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
        ctl_global_interrupts_disable();  // prevent _ticks_? to be modified by an interruption caused by an increment
        _ticks_l = 0; // (IpP<<SHIFT)
        _ticks_r = 0; // (IpP<<SHIFT)
        ctl_global_interrupts_enable();
        continue;
      }
      prevAsserv += 20;

      // get current number of ticks per sampling period
      ctl_global_interrupts_disable();  // prevent _ticks_? to be modified by an interruption caused by an increment
      ticks_l = _ticks_l<<SHIFT; _ticks_l = 0; // (IpP<<SHIFT)
      ticks_r = _ticks_r<<SHIFT; _ticks_r = 0; // (IpP<<SHIFT)
      ctl_global_interrupts_enable();

      // update the motor speed
      // TODO: use the shifted data in the PID of the motor
      motor_controller_update(consigne_l>>SHIFT, ticks_l>>SHIFT, consigne_r>>SHIFT, ticks_r>>SHIFT);

      // update speed and course
      v = (ticks_r + ticks_l)>>1; // (IpP<<SHIFT)
      theta += ticks_r - ticks_l; // (rad.I<<SHIFT)
      // clamp theta to get principal angle
      if(theta > isRPI) {
        theta -= (isRPI<<1);
      }
      else if(theta < -isRPI) {
        theta += (isRPI<<1);
      }

      // update position
      tmp = (long long)v * COS(theta/iD2I(RDIAM));
      x += (int)(tmp>>SHIFT);
      tmp = (long long)v * SIN(theta/iD2I(RDIAM));
      y += (int)(tmp>>SHIFT);

      // distance and angle to the next goal
      sqdist = ((unsigned long long)(gy - y)*(gy - y) + (unsigned long long)(gx - x)*(gx - x))>>SHIFT;

      tmp = lsROUND(D2I(2)*D2I(2)); // 1.5cm position precision
      if(sqdist < tmp) { // we are near the goal
        if(!(step_traj&1) && traj[step_traj>>1].r < isD2I(1)) {
          _consigne_l = _consigne_r = consigne_l = consigne_r = consigne = isRpS2IpP(0);
        }
        else {
          step_traj++;

          if(step_traj&1) {
            gx = traj[(step_traj>>1) + 1].p1_x;
            gy = traj[(step_traj>>1) + 1].p1_y;
            int step_sens = ((long long)traj[step_traj>>1].p1_x - traj[step_traj>>1].p2_x)*((long long)traj[step_traj>>1].c_y - traj[step_traj>>1].p2_y) - ((long long)traj[step_traj>>1].p1_y - traj[step_traj>>1].p2_y)*((long long)traj[step_traj>>1].c_x - traj[step_traj>>1].p2_x) > 0;
            if(step_sens) {
              // set speed a priori
              _consigne_l = (long long)consigne*(traj[step_traj>>1].r + isD2I(RDIAM/2.))/traj[step_traj>>1].r;
              _consigne_r = (long long)consigne*(traj[step_traj>>1].r - isD2I(RDIAM/2.))/traj[step_traj>>1].r;
            }
            else {
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

        //sqdist = ((unsigned long long)(gy - y)*(gy - y) + (unsigned long long)(gx - x)*(gx - x))>>SHIFT;
      }

      // update set point of each motor
      if(step_traj&1) { // circle
        // (I²<<SHIFT)
        tmp = -(( ((x - traj[step_traj>>1].c_x)*(x - traj[step_traj>>1].c_x) + (y - traj[step_traj>>1].c_y)*(y - traj[step_traj>>1].c_y)) - traj[step_traj>>1].r*traj[step_traj>>1].r )>>SHIFT);
        tmp>>=12;
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

        tmp = tmp>>4;
      }

      consigne_l = _consigne_l - tmp;  // (IpP<<SHIFT)
      consigne_r = _consigne_r + tmp;
    }

    if(millis() - prevLed >= 250) {
      prevLed += 250;
      gpio_toggle(1, 24); // LED flashing
    }
  }

  return 0;
}
