#include <lpc214x.h>

#include <gpio.h>
#include <sys_time.h>
#include <eint.h>
#include <ime.h>
#include <pwm.h>

//#define TEST_EXTINT
//#define TEST_PWM

volatile int status_flag = 0;

#ifdef TEST_EXTINT
volatile int irq_count = 0;

void isr_eint0() __attribute__ ((interrupt("IRQ")));
void isr_eint0(){
    status_flag = 1;
    irq_count++;

    SCB_EXTINT = BIT(0); // acknowledges interrupt
    VIC_VectAddr = (unsigned)0; // updates priority hardware
}

void isr_eint3() __attribute__ ((interrupt("IRQ")));
void isr_eint3(){
    status_flag = 2;
    irq_count+=256;

    SCB_EXTINT = BIT(3); // acknowledges interrupt
    VIC_VectAddr = (unsigned)0; // updates priority hardware
}
#endif

int	main(){
    unsigned int prevMillis = 0;
    unsigned int time;
    int state = 0;

    // initialize the I/O
    gpio_init_all();

    // initialize time
    sys_time_init();

#ifdef TEST_PWM
    // inititalize PWM output (shared for every pwm output)
    pwm_init(0, 1024);
    // Write : writes the value 128 on pwm 4
    pwm_enable(4, 128);
#endif

    // sets the I/O {1,24} and {0,31}  (leds) as outputs
    gpio_output(1, 24);
    gpio_output(0, 31);

#ifdef TEST_EXTINT
    eint_init(isr_eint0 /* rising edge on P0.1 */, isr_eint3 /* rising edge on P0.20 */);
    global_IRQ_enable();
#endif

    while(1) {
        sys_time_update();

        time = millis();

        if( (time - prevMillis) > 1000 ) { // every second
            prevMillis = time;  // store current time

            state^=1; // toggle state (led blinking)

#ifdef TEST_PWM
            pwm_update(4, state?512:256);
#endif

            switch(status_flag){
            default:
            case 0:
                gpio_write(1, 24, state);   // writes to output {1,24}
                gpio_write(0, 31, !state);  // writes to output {1,31}
                break;
            case 1:
                gpio_write(1, 24, state);
                break;
            case 2:
                gpio_write(0, 31, !state);
                break;
            }
        }
    }
}
