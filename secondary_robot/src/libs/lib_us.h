/*
 * lib_us.h
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#ifndef LIB_US_H_
#define LIB_US_H_

#include "Arduino.h"

#include "Wire.h"
#include "params.h" //to override adress values
#ifndef US_LOWEST_ADDR
#define US_LOWEST_ADDR 	0xE0  //lowest address of the sensors
#endif
#define US_NB_DEVICES  0

// functions to range using I²C ultrasons
// needs a Wire.begin() in setup()


// send to the sensor the "do range" command, but do no read the result
// sends it in fact to the device at i²c address US_LOWEST_ADDR + nb
int startRange(uint8_t nb) ;

// read the result of a range measurement initiated by startRange
// NB : one must wait for 70ms between a call to startRange and a call to getRangeResult
// sends it in fact to the device at i²c address US_LOWEST_ADDR + nb
// return value in centimeter, -1 if error
uint16_t getRangeResult(uint8_t nb);


// send the "do range" command, waits 70ms (the bad way, with a millis(70)), reads
// the result on the serial port and returns it.
// return value in centimeter, -1 if error
// sends it in fact to the device at i²c address US_LOWEST_ADDR + nb
uint16_t doRange(uint8_t nb) ;


#endif /* LIB_US_H_ */
