#include <targets/LPC2000.h>
#include <ctl_api.h>
#include <math.h>
#include <limits.h>
#include <stdint.h>

#include "gpio.h"
#include "eint.h"
#include "pwm.h"
#include "sys_time.h"
#include "params.h"
#include "trigo.h"
#include "i2c.h"

#include "lib_superBus.h"

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
  static int led_etat = 0;
  unsigned int prev_I2C = 0;
  int ret;
  char buffer[MAX_CHUNK_SIZE];
  uint8_t addr;

  gpio_init_all();  // use fast GPIOs

  sb_init();

// sortie LED
  gpio_output(1, 24);
  gpio_write(1, 24, 0); // green LED on

// init time management
  sys_time_init();

  ctl_global_interrupts_enable();

  sMsg msg;

  msg.header.srcAddr = MYADDRI;
  msg.header.destAddr = ADDRI_MAIN_TURRET;
  msg.header.type = E_DEBUG;
  msg.header.size = 5;
  msg.payload.raw[0] = '\0';
  msg.payload.raw[1] = '\0';
  msg.payload.raw[2] = '\0';
  msg.payload.raw[3] = '\0';
  msg.payload.raw[4] = '\0';

  ret = sb_send(&msg);

// main loop
  while(1) {
    sys_time_update();

    sb_routine();

//    if( millis() - prev_I2C > 2 ){
//      prev_I2C = millis();

//    }
  }

  return 0;
}
