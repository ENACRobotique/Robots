#include <ime.h>

#include <encoder.h>

/**
 * Generic encoder reading for LPC2148
 * Relies on lpc_lib
 *
 * author: Ludovic Lacoste
 *
 * Example of usage:
 *
 *      // Define a global volatile encoder_t variable:
 *      volatile encoder_t enc1;
 *
 *      // Define an interrupt service routine which will handle the true ticks handling
 *      void isr_eint0_enc1() __attribute__ ((interrupt("IRQ")));
 *      void isr_eint0_enc1() {
 *          SCB_EXTINT = BIT(0); // acknowledges interrupt (EINT0 => BIT(0))
 *          VIC_VectAddr = (unsigned) 0; // updates priority hardware
 *
 *          // if there is a 5v level on pin 0.15, nbticks will be incremented
 *          // if there is a 0v level on pin 0.15, nbticks will be decremented
 *          enc1.nbticks += (gpio_read(0, 15) << 1) - 1;
 *      }
 *
 *      // In your main, initialize it:
 *      int main(){
 *          // other initializations...
 *          gpio_input(0, 15);
 *
 *          // initializes the encoder reader on pin0.1, rising edge
 *          encoder_init(&enc1, EINT0, EINT0_P0_1, EINT_RISING_EDGE, isr_eint0_enc1, 2);
 *
 *          // allows irq to be fired
 *          global_IRQ_enable();
 *
 *          while(1){
 *              if(millis() - prev > 20){
 *                  prev += 20;
 *
 *                  // update internal value returned by a call to encoder_get()
 *                  encoder_update(&enc1);
 *
 *                  int processValue = encoder_get(&enc1);
 *
 *                  // do your math with processValue... control loop, ...
 *                  // you can call functions with a pointer to enc1,
 *                  //  they can use it to get the current number of ticks with encoder_get() without altering the internal state
 *              }
 *          }
 *
 *          return 0;
 *      }
 */

/**
 * Initializes the given encoder_t structure and starts ticks collection
 *   the other arguments allows to initializes the underlying external interrupt, see eint.h for more explanations
 *   the handler must increment or decrement the e->nbticks variable according to the current rotation direction
 *   the compiler must be aware the function passed to this function is an isr and the user must take care of the priority hardware and the irq acknoledge
 *
 *   example of handler:
 *      void isr_eint1_enc1() __attribute__ ((interrupt("IRQ")));
 *      void isr_eint1_enc1() {
 *          SCB_EXTINT = BIT(1); // acknowledges interrupt
 *          VIC_VectAddr = (unsigned) 0; // updates priority hardware
 *
 *          enc.nbticks += (gpio_read(BK_CHA_POD1, PIN_CHA_POD1) << 1) - 1; // increment or decrement according to the value of a gpio input channel
 *      }
 */
void encoder_init(encoder_t* e, eEINT eint, eEINT_PINASSIGN eint_pin, eEINT_MODE eint_type, eint_handler eint_h, int eint_prio) {
    e->eint = eint;

    e->nbticks = 0;
    e->nbticks_cache = 0;

    eint_disable(eint);
    eint_assign(eint_pin);
    eint_mode(eint, eint_type);
    eint_register(eint, eint_h, eint_prio);
    eint_enable(eint);
}

/**
 * Updates the internal value returned by encoder_get() with the current sum of ticks since the previous call of encoder_update()
 */
void encoder_update(encoder_t* e) {
    int nbticks;

    global_IRQ_disable();
    nbticks = e->nbticks;
    e->nbticks = 0;
    global_IRQ_enable();

    e->nbticks_cache = nbticks;
}

/**
 * Stops the ticks collection
 */
void encoder_deinit(encoder_t* e) {
    eint_disable(e->eint);
}

/**
 * Returns the cached nbticks updated by a call to encoder_update()
 *   the sign of the value follows the convention used by the interrupt service routine
 */
int encoder_get(encoder_t* e) {
    return e->nbticks_cache;
}
