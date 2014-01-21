#include <lpc214x.h>
#include <stdio.h>
#include <ime.h>
#include <gpio.h>
#include <sys_time.h>
#include <i2c.h>

#define I2C_MASTER 2 // must be equal to 1 or 2

#define I2CADDR_MASTER1 (0x08<<1)
#define I2CADDR_MASTER2 (0x09<<1)
#define I2CSPEED (400000)

int main(void) {
    unsigned int led0_status = 0, led1_status = 0;
    unsigned int led1_prevtime = 0;
    unsigned int time;
    int ret;
    uint8_t buffer[MAX_CHUNK_SIZE];
    uint8_t addr;

    gpio_init_all();  // use fast GPIOs

#if I2C_MASTER == 1
    i2c_init(I2CSPEED, I2CADDR_MASTER1);
#else
    i2c_init(I2CSPEED, I2CADDR_MASTER2);
#endif

    // sortie LED
    gpio_output(1, 24);
    gpio_write(1, 24, led0_status); // green LED on when writing 0

    gpio_output(0, 31);
    gpio_write(0, 31, led1_status); // orange LED on when writing 0

    // init time management
    sys_time_init();

    global_IRQ_enable();

#if I2C_MASTER == 1
    //  ret = i2c_sendchunk(I2CADDR_MASTER2, "Hello World!", 13);
#else
//    ret = i2c_sendchunk(I2CADDR_MASTER1, (uint8_t*)"Hello World!", 13);
    //  ret = i2c_sendchunk(I2CADDR_MASTER1, "Hello World!", 13);
    //  ret = i2c_sendchunk(I2CADDR_MASTER1, "Hello World!", 13);
#endif

    // main loop
    while(1) {
        sys_time_update();
        time = millis();

        ret = i2c_recvchunk(&addr, buffer, sizeof(buffer));
        if(ret > 0){
            gpio_write(1, 24, buffer[0]);

#if I2C_MASTER == 1
//            ret = i2c_sendchunk(I2CADDR_MASTER2, (uint8_t*)"Welcome", 40);
#else
//            ret = i2c_sendchunk(I2CADDR_MASTER1, (uint8_t*)"Welcome", 8);
#endif
        }

#if I2C_MASTER == 1
        if( (time - led1_prevtime) > 100 )
#else
        if( (time - led1_prevtime) > 30 )
#endif
        {
            led1_prevtime = time;

            gpio_write(0, 31, led1_status^=1);

            buffer[0] = led1_status;
            i2c_sendchunk(I2CADDR_MASTER1, buffer, 1);
        }
    }

    return 0;
}
