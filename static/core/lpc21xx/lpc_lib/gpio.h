#ifndef _GPIO_H
#define _GPIO_H

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

#include <targets/LPC2000.h>

static inline void gpio_init_all() {
  SCS = BIT(0) | BIT(1); // Port0&1 GPIOs are fast ones

//  FIO0MASK = 0xFFFFFFFF;
//  FIO1MASK = 0xFFFFFFFF;
}

static inline void gpio_output(int bank, int pin) {
  if(bank==0) {
//    FIO0MASK &= ~BIT(pin);
    FIO0DIR |= BIT(pin);
  }
  else {
    //FIO1MASK &= ~BIT(pin);
    FIO1DIR |= BIT(pin);
  }
}

static inline void gpio_input(int bank, int pin) {
  if(bank==0) {
    //FIO0MASK &= ~BIT(pin);
    FIO0DIR &= ~BIT(pin);
  }
  else {
    //FIO1MASK &= ~BIT(pin);
    FIO1DIR &= ~BIT(pin);
  }
}

static inline int gpio_read(int bank, int pin) {
  if(bank==0) {
    FIO0MASK = ~BIT(pin);
    return (FIO0PIN&BIT(pin))!=0;
  }
  else {
    FIO1MASK = ~BIT(pin);
    return (FIO1PIN&BIT(pin))!=0;
  }
}

static inline void gpio_write(int bank, int pin, int val) {
  if(bank==0) {
    FIO0MASK = ~BIT(pin);
    FIO0PIN = (val!=0)<<pin;
/*    if(val)
      FIO0PIN |= BIT(pin);
    else
      FIO0PIN &= ~BIT(pin);*/
  }
  else {
    FIO1MASK = ~BIT(pin);
    FIO1PIN = (val!=0)<<pin;
/*    if(val)
      FIO1PIN |= BIT(pin);
    else
      FIO1PIN &= ~BIT(pin);*/
  }
}

static inline void gpio_toggle(int bank, int pin) {
  if(bank==0) {
    FIO0MASK = ~BIT(pin);
    FIO0PIN ^= BIT(pin);
  }
  else {
    FIO1MASK = ~BIT(pin);
    FIO1PIN ^= BIT(pin);
  }
}

#endif
