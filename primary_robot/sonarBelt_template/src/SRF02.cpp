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
SRF02::SRF02(uint8_t addr, PoseSonar_t p) {
	_addr = addr;
	_startTime = 0;
	_pose = p;

#ifdef DBG_SRF02
	printf("Created SRF02 sonar: addr = %d, pose = (%d, %d)\n",
			addr, _pose.first, _pose.second);
#endif
}

uint8_t SRF02::getAddr(){
	return _addr;
}
