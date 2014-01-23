
#include "lib_us.h"

// functions to range using I²C ultrasons
// needs a Wire.begin() in setup()


int startRange(uint8_t nb) {
  uint8_t addr = (0xE0 >> 1) + (nb & 0x1);

  // send command to start ranging
  Wire.beginTransmission(addr);
  Wire.write(byte(0x00));      // Write to register 0
  Wire.write(byte(0x51));      // start ranging: measure in inches (0x50), centimeters (0x51) or ping µseconds (0x52)
  return (int)Wire.endTransmission();
}

uint16_t getRangeResult(uint8_t nb) { //retourner plutot la valeur mediane
  uint16_t range;
  uint8_t addr = (0xE0 >> 1) + (nb & 0x1); //70 + nb (with safety)

  // ask data
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));      // Read data starting by register 2
  if(Wire.endTransmission())
    return uint16_t(-1);
  Wire.requestFrom(addr, uint8_t(2));    // Request 2 bytes

  // wait for it
  while(Wire.available() < 2);

  // read, prepare and return data
  range = Wire.read() << 8;
  return range | Wire.read();
}

uint16_t doRange(uint8_t nb) {
  // start ranging
  if(startRange(nb))
    return uint16_t(-1);

  // wait for ranging to happen
  delay(70);

  // get and return result
  return getRangeResult(nb);
}


