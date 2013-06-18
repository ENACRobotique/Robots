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

#define I2C_MASTER 2

int main(void) {
  static int led_etat = 0;
  unsigned int prev_I2C = 0;
  int ret;
  char buffer[MAX_CHUNK_SIZE];
  uint8_t addr;

  gpio_init_all();  // use fast GPIOs

#if I2C_MASTER == 1
  i2c_init(400000, 0x08<<1);
#else
  i2c_init(400000, 0x09<<1);
#endif

// sortie LED
  gpio_output(1, 24);
  gpio_write(1, 24, 0); // green LED on

// init time management
  sys_time_init();

  ctl_global_interrupts_enable();

#if I2C_MASTER == 1
//  ret = i2c_sendchunk(0x09<<1, "Hello World!", 13);
#else
  ret = i2c_sendchunk(0x08<<1, "Hello World!", 13);
//  ret = i2c_sendchunk(0x08<<1, "Hello World!", 13);
//  ret = i2c_sendchunk(0x08<<1, "Hello World!", 13);
#endif

// main loop
  while(1) {
    sys_time_update();

//    if( millis() - prev_I2C > 2 ){
//      prev_I2C = millis();

      ret = i2c_recvchunk(&addr, buffer, sizeof(buffer));
      if(ret > 0){
        gpio_toggle(1, 24);

#if I2C_MASTER == 1
        ret = i2c_sendchunk(0x09<<1, "Welcome", 40);
#else
        ret = i2c_sendchunk(0x08<<1, "Welcome", 8);
#endif
      }
//    }
  }

  return 0;
}
