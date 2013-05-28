/*
 * lib_us.h
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#ifndef LIB_US_H_
#define LIB_US_H_

#include "Arduino.h"

#include "Wire/Wire.h"


// functions to range using I²C ultrasons
// needs a Wire.begin() in setup()


// send to the sensor the "do range" command, but do no read the result
int startRange(uint8_t nb) ;

// read the result of a range measurement initiated by startRange
// NB : one must wait for 70ms between a call to startRange and a call to getRangeResult
// return value in centimeter, -1 if error
uint16_t getRangeResult(uint8_t nb);


// send the "do range" command, waits 70ms (the bad way, with a millis(70)), reads
// the result on the serial port and returns it.
// return value in centimeter, -1 if error
uint16_t doRange(uint8_t nb) ;


#endif /* LIB_US_H_ */
