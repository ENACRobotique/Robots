/*
 * SRF02.h
 *
 *  Created on: Mar 20, 2016
 *      Author: yoyo
 */

#ifndef SRF02_H_
#define SRF02_H_

#include <millis.h>

#define APERTURE_ANGLE 30  // Aperture angle of the detection cone in degrees
#define DELTA (65)  // Amount of time in milliseconds to process the measures

// Registers
#define REG_CMD 0
#define REG_SOFT_REV 0
#define REG_RANGE_H 2
#define REG_RANGE_L 3
#define REG_AUTOTUNE_MIN_H 4
#define REG_AUTOTUNE_MIN_L 5

//// Commands
// Ranging modes
#define CMD_MES_INCH (0x50) // in inches
#define CMD_MES_CM (0x51)  // in centimeters
#define CMD_MES_MS (0x52)  // in milliseconds
// Fake ranging modes
#define CMD_FAKE_MES_INCH (0x56) // in inches
#define CMD_FAKE_MES_CM (0x57)  // in centimeters
#define CMD_FAKE_MES_MS (0x58)  // in milliseconds
// Transmit a burst without ranging
#define CMD_BURST (0x5C)
// Force autotune - same as power up
#define CMD_AUTOTUNE (0x60)
// Change I2C address
#define CMD_CHG_ADD_1 (0xA0)
#define CMD_CHG_ADD_2 (0xA5)
#define CMD_CHG_ADD_3 (0xAA)



class SRF02 {
public:

	/**
	 * @brief SRF02, initializing SRF02 with address and mode
	 * @param ard I2C Device address
	 * @param mode ranging mode
	 */
	SRF02(unsigned int addr);
	unsigned int getAddr();

	private:
		unsigned int _addr;		// address of the sensor
		unsigned long _startTime;	// start time of a ranging, needed
									// to proceed with correct sensor values
};


#endif /* SRF02_H_ */
