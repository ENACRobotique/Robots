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

/*
sTrajEl_t traj_blue[] = {
  {isD2I(7.50), isD2I(100.00), isD2I(88.75), isD2I(112.90), isD2I(82.26), isD2I(90.00), isD2I(105.00), isD2I(8.00), isD2I(16.43)},
  {isD2I(97.58), isD2I(102.44), isD2I(82.42), isD2I(57.56), isD2I(47.37), isD2I(90.00), isD2I(55.00), isD2I(8.00), isD2I(38.89)},
  {isD2I(93.66), isD2I(62.11), isD2I(20.00), isD2I(100.00), isD2I(82.83), isD2I(20.00), isD2I(100.00), isD2I(0.00), isD2I(0.00)}
};
sTrajEl_t traj_red[] = {
  {isD2I(292.50), isD2I(100.00), isD2I(211.25), isD2I(112.90), isD2I(82.26), isD2I(210.00), isD2I(105.00), isD2I(8.00), isD2I(13.83), 0},
  {isD2I(202.00), isD2I(105.00), isD2I(202.00), isD2I(55.00), isD2I(50.00), isD2I(210.00), isD2I(55.00), isD2I(8.00), isD2I(17.91), 0},
  {isD2I(214.95), isD2I(48.72), isD2I(280.00), isD2I(100.00), isD2I(82.83), isD2I(280.00), isD2I(100.00), isD2I(0.00), isD2I(0.00), 0}
};
*/

int main(void) {
  unsigned int prevLed = 0, prevMoteur = 0, state_moteur = 0, prevI2C = 0, prevI2Cpoll = 0;
  static struct i2c_transaction t_tourelle = {.status=I2CTransDone}, t_asserv = {.status=I2CTransSuccess};

  gpio_init_all();  // use fast GPIOs

  i2c0_init(400000, 0 /* master */, NULL, NULL);

// sortie LED
  gpio_output(1, 24);
  gpio_write(1, 24, 0); // LED on

// init time management
  sys_time_init();

  ctl_global_interrupts_enable();

  t_asserv.type = I2CTransTx;
  t_asserv.slave_addr = 0x44;
  t_asserv.len_r = 0;
  t_asserv.len_w = 1;
  t_asserv.buf[0] = 1; // run_traj blue
  t_asserv.status = I2CTransPending;
  i2c0_submit(&t_asserv);

// main loop
  while(1) {
    sys_time_update();

/*    if(millis() - prevI2C >= 100) {
      prevI2C = millis();

      if(t_tourelle.status == I2CTransDone || t_tourelle.status == I2CTransFailed) {
        t_tourelle.type = I2CTransRx;
        t_tourelle.slave_addr = 8<<1;
        t_tourelle.len_r = 12;
        t_tourelle.len_w = 0;

        i2c0_submit(&t_tourelle);
      }
    }*/

    if(millis() - prevLed >= 250) {
      prevLed = millis();

      gpio_toggle(1, 24);
    }
  }

  return 0;
}
