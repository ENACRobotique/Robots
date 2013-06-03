#include <targets/LPC2000.h>
#include <ctl_api.h>
#include <math.h>
#include <limits.h>
#include <string.h>

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
uint8_t traj_extract_idx = 0;
sTrajElRaw_t traj_blue[] = {
  {7.50, 100.00, 88.75, 112.90, 82.26, 90.00, 105.00, 8.00, 16.43, 0, 0},
  {97.58, 102.44, 82.42, 57.56, 47.37, 90.00, 55.00, 8.00, 38.89, 0, 1},
  {93.66, 62.11, 20.00, 100.00, 82.83, 20.00, 100.00, 0.00, 0.00, 0, 2}
};

int main(void) {
  unsigned int prevLed = 0, prevMoteur = 0, state_moteur = 0, prevI2C = 0, prevI2Cpoll = 0;
  static struct i2c_transaction t_tourelle = {.status=I2CTransDone}, t_asserv = {.status=I2CTransSuccess};

  gpio_init_all();  // use fast GPIOs

  i2c0_init(100000, 0 /* master */, NULL, NULL);

// sortie LED
  gpio_output(1, 24);
  gpio_write(1, 24, 0); // LED on

// init time management
  sys_time_init();

  ctl_global_interrupts_enable();

// send position update
  if(t_asserv.status == I2CTransSuccess) {
    t_asserv.type = I2CTransTx;
    t_asserv.slave_addr = 0x44;
    t_asserv.len_r = 0;
    t_asserv.len_w = 4+3*sizeof(float);
    t_asserv.buf[0] = 4; // set position
    *(float *)&t_asserv.buf[4] = traj_blue[0].p1_x; // (cm)
    *(float *)&t_asserv.buf[8] = traj_blue[0].p1_y; // (cm)
    *(float *)&t_asserv.buf[12] = 0.; // (rad)
    t_asserv.status = I2CTransPending;
    i2c0_submit(&t_asserv);
  }

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

    if(traj_extract_idx < sizeof(traj_blue)/sizeof(*traj_blue) && t_asserv.status == I2CTransSuccess) {
      t_asserv.type = I2CTransTx;
      t_asserv.slave_addr = 0x44;
      t_asserv.len_r = 0;
      t_asserv.len_w = 4+sizeof(sTrajElRaw_t);
      t_asserv.buf[0] = 1; // run_traj blue
      memcpy(&t_asserv.buf[4], &traj_blue[traj_extract_idx], sizeof(sTrajElRaw_t));
      t_asserv.status = I2CTransPending;
      i2c0_submit(&t_asserv);

      traj_extract_idx++;
    }

    if(millis() - prevLed >= 250) {
      prevLed = millis();

      gpio_toggle(1, 24);
    }
  }

  return 0;
}
