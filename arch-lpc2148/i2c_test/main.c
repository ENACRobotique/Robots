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

#define MULTI_MASTER

#define I2C_MASTER
#ifndef I2C_MASTER
#define I2C_SLAVE
#endif

unsigned int sum1 = 0, nb1 = 0;
unsigned int sum2 = 0, nb2 = 0;
#if defined(MULTI_MASTER) && defined(I2C_SLAVE)
volatile int can_sendback = 0;
#endif

void i2chnd(struct i2c_transaction *t, void *userp) {
  switch(t->status) {
  case I2CTransRunning: // slave
    if(t->len_r > 0) {
      switch(t->buf[0]) {
      case 0x01:
        t->len_w = 5;
        t->buf[0] = t->buf[1]; // led status
        t->buf[1] = 'l';
        t->buf[2] = 'u';
        t->buf[3] = 'd';
        t->buf[4] = 'o';
        t->buf[5] = '\0';
        gpio_write(1, 24, t->buf[0]);
#if defined(MULTI_MASTER) && defined(I2C_SLAVE)
        can_sendback = 1;
#endif
        break;
      case 0x02:
        t->len_w = 10;
        t->buf[0] = t->buf[1]; // led status
        t->buf[1] = 'd';
        t->buf[2] = 'u';
        t->buf[3] = 'l';
        t->buf[4] = 'o';
        t->buf[5] = '\0';
        gpio_write(0, 31, t->buf[0]);
        break;
      default:
        t->len_w = 1;
        t->buf[0] = 0xff;
        break;
      }
    }
    else { // dafuq?
      t->len_w = 1;
      t->buf[0] = 0xff;
    }
    break;
  case I2CTransSuccess: // master
    if(t->len_r == 5) {
      gpio_write(1, 24, t->buf[0]);
      t->status = I2CTransDone;

      sum1 = (sum1*nb1 + (unsigned int)((int)micros() - (int)t->userp))/(nb1 + 1);
      nb1++;
    }
    else if(t->len_r == 10) {
      gpio_write(0, 31, t->buf[0]);
      t->status = I2CTransDone;

      sum2 = (sum2*nb2 + (unsigned int)((int)micros() - (int)t->userp))/(nb2 + 1);
      nb2++;
    }
    break;
  default:
    break;
  }
}

int main(void) {
  unsigned int prevI2C1 = 0, prevI2C2 = 0;
  static struct i2c_transaction t_1 = {.status=I2CTransDone}, t_2 = {.status=I2CTransDone};
  static int led1_etat = 0, led2_etat = 0;

  gpio_init_all();  // use fast GPIOs

#ifdef I2C_MASTER
#ifdef MULTI_MASTER
  i2c0_init(400000, 0x09<<1 /* slave */, i2chnd, NULL);
#else
  i2c0_init(400000, 0 /* master */, i2chnd, NULL);
#endif
#endif
#ifdef I2C_SLAVE
  i2c0_init(400000, 0x08<<1 /* slave */, i2chnd, NULL);
#endif

// sortie LED
  gpio_output(1, 24);
  gpio_write(1, 24, 0); // green LED on

  gpio_output(0, 31);
  gpio_write(0, 31, 0); // orange LED on

// init time management
  sys_time_init();

  ctl_global_interrupts_enable();

// main loop
  while(1) {
    sys_time_update();

#ifdef I2C_MASTER
    if(millis() - prevI2C1 >= 40) {
      prevI2C1 += 40;

      if(t_1.status == I2CTransDone) {
        t_1.type = I2CTransTxRx;
        t_1.buf[0] = 0x01;
        t_1.buf[1] = (led1_etat^=1);
        t_1.slave_addr = 8<<1;
        t_1.len_r = 5;
        t_1.len_w = 10;
        t_1.userp = (void *)micros();

        i2c0_submit(&t_1);
      }
    }
#ifndef MULTI_MASTER
    if(millis() - prevI2C2 >= 130) {
      prevI2C2 += 130;

      if(t_2.status == I2CTransDone) {
        t_2.type = I2CTransTxRx;
        t_2.buf[0] = 0x02;
        t_2.buf[1] = (led2_etat^=1);
        t_2.slave_addr = 8<<1;
        t_2.len_r = 10;
        t_2.len_w = 20;
        t_2.userp = (void *)micros();

        i2c0_submit(&t_2);
      }
    }
#endif
#endif

#if defined(MULTI_MASTER) && defined(I2C_SLAVE)
    if(millis() - prevI2C2 >= 130) {
      prevI2C2 += 130;

      if(can_sendback && t_2.status == I2CTransDone) {
        t_2.type = I2CTransTxRx;
        t_2.buf[0] = 0x02;
        t_2.buf[1] = (led2_etat^=1);
        t_2.slave_addr = 9<<1;
        t_2.len_r = 10;
        t_2.len_w = 20;
        t_2.userp = (void *)micros();

        i2c0_submit(&t_2);
      }
    }
#endif
  }

  return 0;
}
