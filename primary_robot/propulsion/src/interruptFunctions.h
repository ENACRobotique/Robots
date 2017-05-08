/*
 * interruptFunctions.h
 *
 *  Created on: 6 mai 2017
 *      Author: fabien
 */

#ifndef INTERRUPTFUNCTIONS_H_
#define INTERRUPTFUNCTIONS_H_

void setupInterrupts();

void isrLeft();
void isrRight();

void isrTiretteRising();
void isrTiretteFalling();
void isrColorRising();
void isrColorFalling();

#endif /* INTERRUPTFUNCTIONS_H_ */
