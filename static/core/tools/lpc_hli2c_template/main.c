#include <lpc214x.h>
#include <stdio.h>
#include <ime.h>
#include <gpio.h>
#include <sys_time.h>
#include <i2c.h>

#define I2C_MASTER 1 // must be equal to 1 or 2
//#define I2C_POLL_PERIOD 200 // comment if necessary

int main(void) {
    unsigned int led0_status = 0, led1_status = 0;
    unsigned int prev_led1 = 0;
#ifdef I2C_POLL_PERIOD
    unsigned int prev_I2C = 0;
#endif
    unsigned int time;
    int ret;
    uint8_t buffer[MAX_CHUNK_SIZE];
    uint8_t addr;

    gpio_init_all();  // use fast GPIOs

#if I2C_MASTER == 1
    i2c_init(400000, 0x04<<1);
#else
    i2c_init(400000, 0x09<<1);
#endif

    // sortie LED
    gpio_output(0, 31);
    gpio_write(0, 31, 0); // orange LED on

    gpio_output(1, 24);
    gpio_write(1, 24, 0); // green LED on

    // init time management
    sys_time_init();

    global_IRQ_enable();

#if I2C_MASTER == 1
    //  ret = i2c_sendchunk(0x09<<1, "Hello World!", 13);
#else
    ret = i2c_sendchunk(0x04<<1, (uint8_t*)"Hello World!", 13);
    //  ret = i2c_sendchunk(0x08<<1, "Hello World!", 13);
    //  ret = i2c_sendchunk(0x08<<1, "Hello World!", 13);
#endif

    // main loop
    while(1) {
        sys_time_update();
        time = millis();

#if I2C_MASTER == 1
        if( (time - prev_led1) > 100 )
#else
        if( (time - prev_led1) > 500 )
#endif
        {
            prev_led1 = time;

            gpio_write(0, 31, led1_status^=1);
        }

#ifdef I2C_POLL_PERIOD
        if( (time - prev_I2C) > I2C_POLL_PERIOD ){
            prev_I2C = time;
#endif
            ret = i2c_recvchunk(&addr, buffer, sizeof(buffer));
            if(ret > 0){
                gpio_write(1, 24, led0_status^=1);

#if I2C_MASTER == 1
//                ret = i2c_sendchunk(0x09<<1, (uint8_t*)"Welcome", 40);
#else
//                ret = i2c_sendchunk(0x08<<1, (uint8_t*)"Welcome", 8);
#endif
            }
#ifdef I2C_POLL_PERIOD
        }
#endif
    }

    return 0;
}
