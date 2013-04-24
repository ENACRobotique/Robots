#include <targets/LPC2000.h>

unsigned char _addr;

void _i2c0_isr() { // SI bit is set in I2C0CONSET => state change
  switch(I2STAT) { // get new status
  // bus free, start condition emitted, can send slave address
  case 0x08:
    I2DAT = _addr<<1 | 0; // bit0 is direction, 0 is write
    I2CONCLR |= 0x20; // clear STA bit in I2C0CONSET
    break;

  case 0x10:

  // ack received from slave (no slave mode AA=0)
  case 0x18:
  case 0x20:
  case 0x38:
    break;

  // ack received from master (slave mode enabled AA=1)
  case 0x68:
  case 0x78:
  case 0xB0:
    break;
  }

  I2CONCLR |= 0x4; // clear SI bit in I2C0CONSET
}

void i2c0_init() {
  // SCL0 on P0.2
  PINSEL0 &= ~(3<<4);
  PINSEL0 |= 1<<4;

  // SDA0 on P0.3
  PINSEL0 &= ~(3<<6);
  PINSEL0 |= 1<<6;

  // bind I2C0 (irq9) to interrupt priority 11
  VICVectCntl11 = 0x20 | 9;
  VICVectAddr11 = _i2c0_isr;

  // enable I2C0 interrupt
  VICIntEnable |= 1<<9;
}

void i2c0_slave_init() {
  i2c0_init();

}

void i2c0_master_init() {
  i2c0_init();

}

void i2c0_master_transmit(unsigned char addr, ) {
  
}
