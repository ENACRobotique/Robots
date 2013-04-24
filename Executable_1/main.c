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

int A = isD2I(50.);
int B = isD2I(170.);
int i_traj = 0;

int main(void) {
  unsigned int prevAsserv=0, prevLed=0;
  int ticks_l, ticks_r;
  int consigne_l = 0;
  int consigne_r = 0;
  const int consigne = isRpS2IpP(0.5);
  long long tmp;

  static int x = isD2I(INIT_X), y = isD2I(INIT_Y), theta = iROUND(INIT_THETA*D2I(RDIAM)*dSHIFT);
  int v, alpha;
  unsigned long long sqdist;

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
      sqdist = ((unsigned long long)(B - y)*(B - y) + (unsigned long long)(A - x)*(A - x))>>SHIFT;

      tmp = lsROUND(D2I(1.5)*D2I(1.5)); // 1.5cm position precision
      if(sqdist < tmp) { // near to the goal
        switch(i_traj) {
        default:
        case 0:
          i_traj=1;
          A += isD2I(20.);
          B += isD2I(20.);
          break;
        case 1:
          i_traj=2;
          A += isD2I(20.);
          //B -= isD2I(20.);
          break;
        case 2:
          i_traj=3;
          A += isD2I(20.);
          B -= isD2I(20.);
          break;
        case 3:
          i_traj=0;
          A += isD2I(20.);
//          B -= isD2I(20.);
          break;
        }

        sqdist = ((unsigned long long)(B - y)*(B - y) + (unsigned long long)(A - x)*(A - x))>>SHIFT;
      }
      if((A - x)>0) {
        alpha = ATAN2(B - y, A - x);
      }
      else if((B - y)>0) {
        alpha = isPI + ATAN2(B - y, A - x);
      }
      else {
        alpha = -isPI + ATAN2(B - y, A - x);
      }

      // update set point of each motor
      tmp = iD2I(RDIAM) * alpha - theta;  // (rad.I<<SHIFT)

      consigne_l = consigne - (tmp>>4);  // (IpP<<SHIFT)
      consigne_r = consigne + (tmp>>4);
    }

    if(millis() - prevLed >= 250) {
      prevLed += 250;
      gpio_toggle(1, 24); // LED flashing
    }
  }

  return 0;
}
