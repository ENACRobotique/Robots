#include <targets/LPC2000.h>
#include <ctl_api.h>
#include <math.h>
#include <limits.h>

#include "gpio.h"
#include "eint.h"
#include "pwm.h"
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

void mybreak_i() {
  static int i = 0;
  i++;
}

void hndi2c(struct i2c_transaction *t, void *userp) {
  t->len_w = 0;
/*  t->buf[0] = 1;
  t->buf[1] = 5;*/
}

//#define TEST_MONTEE
#ifdef TEST_MONTEE
//#define USE_PWM_MONTEE
#endif

int main(void) {
  unsigned int prevLed = 0, prevMoteur = 0, state_moteur = 0, prevI2C = 0;

  gpio_init_all();  // use fast GPIOs

  i2c0_init(100000, 0x42, hndi2c, NULL); // slave

#ifdef TEST_MONTEE
#ifdef USE_PWM_MONTEE
  pwm_init(1, 1024);  // 29.3kHz update rate => not audible
#endif

// moteur droit
#ifdef USE_PWM_MONTEE
  pwm_enable(1, 1023);
#else
  gpio_output(0, 0); gpio_write(0, 0, 1);
#endif
  gpio_output(0, 11); gpio_write(0, 11, 1);
  gpio_output(0, 12); gpio_write(0, 12, 1);

// moteur gauche
#ifdef USE_PWM_MONTEE
  pwm_enable(2, 1023);
#else
  gpio_output(0, 7); gpio_write(0, 7, 1);
#endif
  gpio_output(0, 13); gpio_write(0, 13, 1);
  gpio_output(0, 14); gpio_write(0, 14, 1);
#endif

// sortie LED
  gpio_output(1, 24);
  gpio_write(1, 24, 0); // LED on

// init time management
  sys_time_init();

  ctl_global_interrupts_enable();

// main loop
  while(1) {
    sys_time_update();

#ifdef TEST_MONTEE
    if(millis() - prevMoteur >= 2000) {
      prevMoteur += 2000;

      switch(state_moteur) {
      case 0:
        // moteur droit
        gpio_write(0, 11, 1);
        gpio_write(0, 12, 0);
#ifdef USE_PWM_MONTEE
        pwm_update(1, 0);
#else
        gpio_write(0, 0, 0);
#endif

        // moteur gauche
        gpio_write(0, 13, 1);
        gpio_write(0, 14, 0);
#ifdef USE_PWM_MONTEE
        pwm_update(2, 0);
#else
        gpio_write(0, 7, 0);
#endif
        break;
      case 1:
        // moteur droit
        gpio_write(0, 11, 0);
        gpio_write(0, 12, 0);
#ifdef USE_PWM_MONTEE
        pwm_update(1, 1023);
#else
        gpio_write(0, 0, 1);
#endif

        // moteur gauche
        gpio_write(0, 13, 0);
        gpio_write(0, 14, 0);
#ifdef USE_PWM_MONTEE
        pwm_update(2, 1023);
#else
        gpio_write(0, 7, 1);
#endif
        break;
      case 2:
        // moteur droit
        gpio_write(0, 11, 0);
        gpio_write(0, 12, 1);
#ifdef USE_PWM_MONTEE
        pwm_update(1, 0);
#else
        gpio_write(0, 0, 0);
#endif

        // moteur gauche
        gpio_write(0, 13, 0);
        gpio_write(0, 14, 1);
#ifdef USE_PWM_MONTEE
        pwm_update(2, 0);
#else
        gpio_write(0, 7, 0);
#endif
        break;
      case 3:
        // moteur droit
        gpio_write(0, 11, 0);
        gpio_write(0, 12, 0);
#ifdef USE_PWM_MONTEE
        pwm_update(1, 1023);
#else
        gpio_write(0, 0, 1);
#endif

        // moteur gauche
        gpio_write(0, 13, 0);
        gpio_write(0, 14, 0);
#ifdef USE_PWM_MONTEE
        pwm_update(2, 1023);
#else
        gpio_write(0, 7, 1);
#endif
        break;
      default:
        state_moteur = 0;
        break;
      }

      state_moteur = (state_moteur + 1)&3;
    }
#endif

    if(millis() - prevLed >= 250) {
      prevLed += 250;
      gpio_toggle(1, 24); // LED flashing
    }
  }

  return 0;
}
