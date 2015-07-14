/*
 * state_Scan_i2c.h
 *
 *  Created on: 2015 juillet 13
 *      Author: Fab
 */

#ifndef STATE_SCAN_I2C_H_
#define STATE_SCAN_I2C_H_

#include "state_types.h"

extern sState sScan_i2c;
void scan_i2c(char success[], char unknow_error[]);
void display(byte add, char success[], char unknow_error[]);
#endif /* STATE_SCAN_I2C_ */
