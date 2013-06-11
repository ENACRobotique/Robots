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

#include "lib_superBus.h"
#include "lib_checksum.h"
#include "messages.h"
#include "network_cfg.h"

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

int main(void) {
  unsigned int prevLed = 0, prevI2C = 0;
  sMsg tmp;
  int ret;

  gpio_init_all();  // use fast GPIOs

  sb_init();

// sortie LED
  gpio_output(1, 24);
  gpio_write(1, 24, 0); // LED on

  gpio_output(0, 31);
  gpio_write(0, 31, 0); // LED on

// init time management
  sys_time_init();

  ctl_global_interrupts_enable();

// send position update
/*  if(t_asserv.status == I2CTransSuccess) {
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

  tmp.header.destAddr = ADDRI_MAIN_PROP;
  tmp.header.size = 4+3+sizeof(float);
  tmp.header.type = E_DEBUG;
  tmp.payload.raw[0] = 4; // set position
  *(float *)&t_asserv.buf[4] = traj_blue[0].p1_x; // (cm)
  *(float *)&t_asserv.buf[8] = traj_blue[0].p1_y; // (cm)
  *(float *)&t_asserv.buf[12] = 0.; // (rad)
  sb_send(&tmp);
*/

  sb_printDbg(ADDRX_DEBUG, "start candle", 0, 0);

// main loop
  while(1) {
    sys_time_update();

    sb_routine();

    if(sb_receive(&tmp) > 0){
      gpio_write(0, 31, 1);//tmp.payload.raw[0]);

      tmp.header.srcAddr = MYADDRI;
      tmp.header.destAddr = ADDRX_DEBUG;//ADDRX_DEBUG;
      tmp.header.size = 13;
      tmp.header.type = E_DEBUG;
      strcpy(tmp.payload.debug.msg, "Hello world!"),
      ret = sb_send(&tmp);
    }

/*    if(millis() - prevI2C >= 100) {
      prevI2C = millis();

      if(t_tourelle.status == I2CTransSuccess) {
        t_tourelle.type = I2CTransTx;
        t_tourelle.slave_addr = 8<<1;
        t_tourelle.len_r = 12;
        t_tourelle.len_w = 0;

        i2c0_submit(&t_tourelle);
      }
    }

    if(
      traj_extract_idx < sizeof(traj_blue)/sizeof(*traj_blue) &&
      t_asserv.status == I2CTransSuccess
    ) {
      t_asserv.type = I2CTransTx;
      t_asserv.slave_addr = 0x44;
      t_asserv.len_r = 0;
      t_asserv.len_w = 4+sizeof(sTrajElRaw_t);
      t_asserv.buf[0] = 1; // run_traj blue
      memcpy(&t_asserv.buf[4], &traj_blue[traj_extract_idx], sizeof(sTrajElRaw_t));
      t_asserv.status = I2CTransPending;
      i2c0_submit(&t_asserv);

      traj_extract_idx++;
    }*/

    if(millis() - prevLed >= 250) {
      prevLed = millis();

      gpio_toggle(1, 24);
//      gpio_write(0, 31, gpio_read(1, 24));
    }
  }

  return 0;
}
