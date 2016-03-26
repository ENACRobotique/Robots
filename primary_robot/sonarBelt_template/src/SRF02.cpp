/*
 * SRF02.cpp
 *
 *  Created on: Mar 20, 2016
 *      Author: yoyo
 */

#include "SRF02.h"


/**
 * @brief SRF02 CTor, initializing SRF02 with address and mode
 * @param ard I2C Device address
 * @param mode ranging mode
 */
SRF02::SRF02(unsigned int addr) {
	_addr = addr;
	_startTime = 0;
}

unsigned int SRF02::getAddr(){
	return _addr;
}
