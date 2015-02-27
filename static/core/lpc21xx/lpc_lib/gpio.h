#ifndef _GPIO_H
#define _GPIO_H

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

#include <lpc214x.h>

static inline void gpio_init_all() {
  SCB_SCS = BIT(0) | BIT(1); // Port0&1 GPIOs are fast ones
}

static inline void gpio_enable(int bank, int pin) {
  if (bank == 0) {
    if (pin <= 15) {
      PCB_PINSEL0 &= ~(3 << (pin << 1));
    }
    else {
      PCB_PINSEL1 &= ~(3 << ((pin - 16) << 1));
    }
  }
}

static inline void gpio_input(int bank, int pin);

static inline void gpio_disable(int bank, int pin) {
  gpio_input(bank, pin);
}

static inline void gpio_output(int bank, int pin) {
  if(bank==0) {
    GPIO0_FIODIR |= BIT(pin);
  }
  else {
    GPIO1_FIODIR |= BIT(pin);
  }
}

static inline void gpio_input(int bank, int pin) {
  if(bank==0) {
    GPIO0_FIODIR &= ~BIT(pin);
  }
  else {
    GPIO1_FIODIR &= ~BIT(pin);
  }
}

static inline int gpio_read(int bank, int pin) {
  if(bank==0) {
    GPIO0_FIOMASK = ~BIT(pin);
    return (GPIO0_FIOPIN&BIT(pin))!=0;
  }
  else {
    GPIO1_FIOMASK = ~BIT(pin);
    return (GPIO1_FIOPIN&BIT(pin))!=0;
  }
}

static inline void gpio_write(int bank, int pin, int val) {
  if(bank==0) {
    GPIO0_FIOMASK = ~BIT(pin);
    GPIO0_FIOPIN = (val!=0)<<pin;
  }
  else {
    GPIO1_FIOMASK = ~BIT(pin);
    GPIO1_FIOPIN = (val!=0)<<pin;
  }
}

static inline void gpio_toggle(int bank, int pin) {
  if(bank==0) {
    GPIO0_FIOMASK = ~BIT(pin);
    GPIO0_FIOPIN ^= BIT(pin);
  }
  else {
    GPIO1_FIOMASK = ~BIT(pin);
    GPIO1_FIOPIN ^= BIT(pin);
  }
}

#endif
