/*
 * lib_IHM.h
 *
 *  Created on: 2015
 *      Author: Fab
 */

#ifndef LIB_IHM_H_
#define LIB_IHM_H_

//#include <state_types.h>
#include "Encoder.h"
#include "../../../../core/arduino/libraries/LiquidCrystal/LiquidCrystal.h"

extern Encoder myEnc;

void afficher(int col, int row, const char * format...) __attribute__((format (printf, 3, 4)));
void afficher(const char * format...) __attribute__((format (printf, 1, 2)));
void eraseLine(int row);
void clearLcd();
void startLcd();

#endif /* LIB_IHM_H_ */
