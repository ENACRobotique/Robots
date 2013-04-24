#include <targets/LPC2000.h>

#include "gpio.h"

void gpio_init_all() {
  SCS |= BIT(0) | BIT(1); // Port0&1 GPIOs are fast ones
}

void gpio_output(int bank, int pin) {
  if(bank==0) {
    FIO0MASK &= ~BIT(pin);
    FIO0DIR |= BIT(pin);
  }
  else {
    FIO1MASK &= ~BIT(pin);
    FIO0DIR |= BIT(pin);
  }
}

void gpio_input(int bank, int pin) {
  if(bank==0) {
    FIO0MASK &= ~BIT(pin);
    FIO0DIR &= ~BIT(pin);
  }
  else {
    FIO1MASK &= ~BIT(pin);
    FIO0DIR &= ~BIT(pin);
  }
}

int gpio_read(int bank, int pin) {
  if(bank==0)
    return !!(FIO0PIN&BIT(pin));
  else
    return !!(FIO1PIN&BIT(pin));
}

void gpio_write(int bank, int pin, int val) {
  if(bank==0) {
    if(val)
      FIO0PIN |= BIT(pin);
    else
      FIO0PIN &= ~BIT(pin);
  }
  else {
    if(val)
      FIO1PIN |= BIT(pin);
    else
      FIO1PIN &= ~BIT(pin);
  }
}

void gpio_toggle(int bank, int pin) {
  if(bank==0)
    FIO0PIN ^= BIT(pin);
  else
    FIO1PIN ^= BIT(pin);
}
