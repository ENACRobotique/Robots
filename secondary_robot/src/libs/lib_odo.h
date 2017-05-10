/*
 * lib_odo.h
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#ifndef LIB_ODO_H_
#define LIB_ODO_H_

#include "Arduino.h"


void odoIsr();

//initialise the pins and sets the ISR
//requires : nothing
void odoInitHard(int pinInt[], int pinSen[]);

//remove the ISR
void odoDeinit();

//returns the (signed) number of increments between now and the previous call to odo_read, and resets this number
int odoRead(int index_motor);

//
void razAccumulators();
long readAccumulators(int motor_index);

#endif /* LIB_ODO_H_ */
